"""Independent moving-board finder: RGB/static layout in, corners/code out.

No GT, header, expected tick, expected epoch, or nearest-code correction input.
OpenCV coordinates are converted to the explicit pixel-center convention.
"""
import math

import cv2
import numpy as np

from marker_codec import decode_marker

DEFAULT_VISION_LAYOUT = {
    "outer_ring_cells":[16,14],"canonical_cell_px":12,
    "payload_layout":{"x":24,"y":24,"cell_px":12,"cols":12,"rows":10},
    "minimum_side_px":48,"minimum_area_px":3000,
    "minimum_reference_contrast":60,"maximum_edge_fit_rms_px":0.8,
}


def _order_quad(points):
    points = np.asarray(points,dtype=np.float64).reshape(4,2)
    center = points.mean(axis=0)
    points = points[np.argsort(np.arctan2(points[:,1]-center[1],points[:,0]-center[0]))]
    points = np.roll(points,-np.argmin(points.sum(axis=1)),axis=0)
    return points


def _intersection(first,second):
    # Each line is its unit normal n and n.point = constant.
    mat = np.array([first[0],second[0]])
    if abs(np.linalg.det(mat))<0.05:
        raise ValueError("adjacent edges nearly parallel")
    return np.linalg.solve(mat,np.array([first[1],second[1]]))


def _refine_outer_edges(gray,quad,min_contrast,max_rms):
    """Find black-to-white transitions and fit four fixed outer-border edges."""
    lines=[]; quality=[]
    offsets=np.linspace(-4,4,33,dtype=np.float32)
    for i in range(4):
        start,end=quad[i],quad[(i+1)%4]
        tangent=end-start; length=np.linalg.norm(tangent); tangent/=length
        outward=np.array([tangent[1],-tangent[0]])
        samples=[]; contrasts=[]
        for fraction in np.linspace(.12,.88,24):
            base=start+(end-start)*fraction
            xy=base[None,:]+offsets[:,None]*outward[None,:]
            vals=cv2.remap(gray,xy[:,0].astype(np.float32)[None,:],
                           xy[:,1].astype(np.float32)[None,:],cv2.INTER_LINEAR,
                           borderMode=cv2.BORDER_CONSTANT,borderValue=0).reshape(-1)
            dark=float(np.median(vals[:7])); bright=float(np.median(vals[-7:]))
            contrast=bright-dark
            if contrast<min_contrast: continue
            half=(dark+bright)/2
            crossings=np.where((vals[:-1]<=half)&(vals[1:]>half))[0]
            if len(crossings)!=1: continue
            j=int(crossings[0]); part=(half-vals[j])/(vals[j+1]-vals[j])
            distance=float(offsets[j]+part*(offsets[j+1]-offsets[j]))
            samples.append(base+outward*distance); contrasts.append(contrast)
        if len(samples)<12:
            raise ValueError("insufficient clean outer-edge samples")
        pts=np.asarray(samples)
        vx,vy,x,y=cv2.fitLine(pts.astype(np.float32),cv2.DIST_HUBER,0,.001,.001).reshape(-1)
        normal=np.array([-vy,vx],dtype=float); constant=float(normal@np.array([x,y]))
        rms=float(np.sqrt(np.mean((pts@normal-constant)**2)))
        if rms>max_rms: raise ValueError("outer-edge fit residual exceeds fixed limit")
        lines.append((normal,constant));quality.append({"sample_count":len(samples),"fit_rms_px":rms,"median_contrast":float(np.median(contrasts))})
    refined=np.array([_intersection(lines[(i-1)%4],lines[i]) for i in range(4)])
    if np.max(np.linalg.norm(refined-quad,axis=1))>4:
        raise ValueError("outer-edge refinement moved too far")
    return refined,quality


def detect_board(rgb,search_roi_xyxy,layout_metadata=None):
    """Return only independently visible board identity and fixed outer corners.

    `layout_metadata` is either the static vision_layout or full board metadata.
    A board_id may restrict the marker role, but no acquisition-state hint exists.
    """
    config=dict(DEFAULT_VISION_LAYOUT)
    supplied=layout_metadata or {}
    if "vision_layout" in supplied: supplied=supplied["vision_layout"]
    config.update(supplied)
    image=np.asarray(rgb)
    if image.ndim!=3 or image.shape[2]<3 or image.dtype!=np.uint8:
        return {"status":"invalid_image","valid":False,"reason":"expected uint8 RGB"}
    x0,y0,x1,y1=map(int,search_roi_xyxy)
    if not (0<=x0<x1<=image.shape[1] and 0<=y0<y1<=image.shape[0]):
        return {"status":"invalid_roi","valid":False,"reason":"static ROI outside image"}
    gray=image[:,:,:3].mean(axis=2).astype(np.float32)
    crop=gray[y0:y1,x0:x1].astype(np.uint8)
    threshold,binary=cv2.threshold(crop,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
    contours,hierarchy=cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    proposals=[];candidates=[]; rejected=[]
    if hierarchy is None:
        return {"status":"not_detected","valid":False,"candidates":[],"reason":"no dark contours"}
    for index,contour in enumerate(contours):
        area=float(cv2.contourArea(contour))
        if area<float(config["minimum_area_px"]):continue
        child=int(hierarchy[0,index,2])
        if child<0 or cv2.contourArea(contours[child])<area*.40:continue
        perimeter=cv2.arcLength(contour,True)
        poly=cv2.approxPolyDP(contour,.015*perimeter,True)
        if len(poly)!=4 or not cv2.isContourConvex(poly):continue
        quad=_order_quad(poly.reshape(4,2)+[x0,y0])
        if min(np.linalg.norm(quad-np.roll(quad,-1,axis=0),axis=1))<config["minimum_side_px"]:continue
        if any(np.max(np.linalg.norm(quad-q,axis=1))<3 for q in proposals):continue
        proposals.append(quad)
        try:
            refined,quality=_refine_outer_edges(gray,quad,float(config["minimum_reference_contrast"]),float(config["maximum_edge_fit_rms_px"]))
        except ValueError as exc:
            rejected.append({"reason":str(exc),"initial_quad_index_xy":quad.tolist()});continue
        width,height=[int(v*config["canonical_cell_px"]) for v in config["outer_ring_cells"]]
        destination=np.array([[-.5,-.5],[width-.5,-.5],[width-.5,height-.5],[-.5,height-.5]],dtype=np.float32)
        orientation_results=[]
        for turn in range(4):
            source=np.roll(refined,-turn,axis=0).astype(np.float32)
            homography=cv2.getPerspectiveTransform(source,destination)
            rectified=cv2.warpPerspective(image,homography,(width,height),flags=cv2.INTER_LINEAR,
                                          borderMode=cv2.BORDER_CONSTANT,borderValue=0)
            decoded=decode_marker(rectified,config["payload_layout"])
            orientation_results.append(decoded)
            if decoded.get("valid"):
                if "board_id" in config and decoded["camera_id"]!=int(config["board_id"]):
                    continue
                candidates.append({"valid":True,"status":"detected","decoded":decoded,
                                   "board_id":decoded["camera_id"],"corners_image_xy":(source+.5).tolist(),
                                   "corner_order":["top_left","top_right","bottom_right","bottom_left"],
                                   "homography_image_index_to_rectified_index":homography.tolist(),
                                   "rectified_shape_hw":[height,width],"edge_quality":quality,
                                   "rotation_quarters":turn,"threshold_otsu":float(threshold),
                                   "pixel_coordinates":"centers at column+0.5,row+0.5; corners are fixed black ring outer edges"})
        if not any(v.get("valid") for v in orientation_results):
            rejected.append({"reason":"all four independent orientations fail decoder","corners_image_xy":(refined+.5).tolist(),"orientation_decodes":orientation_results})
    if len(candidates)==1:
        return dict(candidates[0],candidate_count=1,rejected_candidates=rejected)
    if len(candidates)>1:
        return {"valid":False,"status":"ambiguous","reason":"multiple independently valid board candidates","candidates":candidates,"candidate_count":len(candidates)}
    return {"valid":False,"status":"decode_failed" if proposals else "not_detected",
            "reason":"no uniquely decodable fixed-border board","candidate_count":0,
            "proposal_count":len(proposals),"rejected_candidates":rejected}


def _synthetic_test():
    """Algorithm plumbing only: this does not prove any Isaac/GPU behavior."""
    from marker_codec import encode_marker
    cell=10; grid=np.ones((16,18),dtype=np.uint8)
    grid[1:15,1:17]=0;grid[2:14,2:16]=1
    grid[3:13,3:15]=encode_marker(0x56781234,28,102)
    tile=np.repeat(np.repeat(grid,cell,axis=0),cell,axis=1)
    tile=np.repeat(np.where(tile[...,None],225,15).astype(np.uint8),3,axis=2)
    src=np.array([[10,10],[170,10],[170,150],[10,150]],np.float32)-.5
    wanted=np.array([[65,35],[242,49],[231,203],[48,181]],np.float32)
    transform=cv2.getPerspectiveTransform(src,wanted-.5)
    picture=cv2.warpPerspective(tile,transform,(320,240),flags=cv2.INTER_LINEAR,borderValue=(80,80,80))
    result=detect_board(picture,[0,0,320,240],dict(DEFAULT_VISION_LAYOUT,board_id=102))
    assert result["valid"],result
    assert result["decoded"]["tick"]==28 and result["decoded"]["epoch"]==0x56781234
    error=np.linalg.norm(np.array(result["corners_image_xy"])-wanted,axis=1)
    assert max(error)<1.0,(error,result)
    grid[7,7]^=1
    badtile=np.repeat(np.repeat(grid,cell,axis=0),cell,axis=1)
    badtile=np.repeat(np.where(badtile[...,None],225,15).astype(np.uint8),3,axis=2)
    bad=cv2.warpPerspective(badtile,transform,(320,240),flags=cv2.INTER_LINEAR,borderValue=(80,80,80))
    rejected=detect_board(bad,[0,0,320,240],dict(DEFAULT_VISION_LAYOUT,board_id=102))
    assert not rejected["valid"],rejected
    print("Synthetic perspective plumbing passed; fixed-corner max error",float(max(error)),"px; corrupted CRC rejected. This is not GPU evidence.")


if __name__=="__main__":
    _synthetic_test()
