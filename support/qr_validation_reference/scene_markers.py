"""Optional 3D markers, captured by the original camera RenderProducts.

Revision 4 uses fixed-color white quads and a fixed black backing. Only each
changed cell's USD Xform scale changes, moving it before/behind the backing.
"""
from copy import deepcopy

from pxr import Gf, Sdf, UsdGeom, UsdShade

from marker_codec import DEFAULT_LAYOUT, WARMUP_TICK, encode_marker, encoding_metadata


def _material(stage, path, emission):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(emission))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _quad(stage, path, points, material):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4]); mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none); mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr([Gf.Vec3f(*(min(p[i] for p in points) for i in range(3))),
                           Gf.Vec3f(*(max(p[i] for p in points) for i in range(3)))])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh


class SceneMarkers:
    def __init__(self, stage, cameras, epoch, cell_px=12):
        if cell_px not in (12,16):raise ValueError('Reviewed marker cell sizes are 12 or 16 pixels')
        self.epoch = int(epoch)
        self.handles = []
        self.metadata = encoding_metadata()
        self.metadata.update(enabled=True, epoch=self.epoch, optical_depth_m=0.5,
                             representation_revision=4,
                             geometry="constant white quads scaled about camera optical origin in front of/behind fixed black backing; same RenderProduct as drone",
                             material="constant white/black UsdPreviewSurface self-emission; diffuse black; materials unchanged after creation",
                             update="USD Xform uniform scale of changed cells only; no mesh points/material/texture updates, extra renders or readbacks",
                             layout_selection="use cameras[camera_id].layout; top-right placement differs by image width",
                             white_quad_depths_m={"bit_1":0.49, "bit_0":0.51},
                             white_quad_scale={"bit_1":0.98, "bit_0":1.02},
                             coordinates="top-left pixel edge convention; central half-cell sampled offline",
                             cameras={})
        root = "/Stage1ValidationMarkers"
        UsdGeom.Xform.Define(stage, root)
        black = _material(stage, root + "/BlackMaterial", 0.002)
        white = _material(stage, root + "/WhiteMaterial", 0.85)
        for index, cam in enumerate(cameras, 1):
            layout = dict(DEFAULT_LAYOUT, x=int(cam["width"])-12-12*cell_px, cell_px=cell_px)
            if index == 1: self.metadata["layout"] = deepcopy(layout)
            camera_root = root + "/" + cam["camera_id"]
            camera_xf = UsdGeom.Xform.Define(stage, camera_root)
            optical = Gf.Matrix4d(*[v for row in cam["T_world_optical_row"] for v in row])
            camera_xf.AddTransformOp().Set(optical)
            fx, fy, cx, cy = (cam["k"][i] for i in (0, 4, 2, 5))
            def point(u, v):
                return Gf.Vec3f((u-cx)*0.5/fx, (v-cy)*0.5/fy, 0.5)
            operations = []
            for row in range(layout["rows"]):
                for col in range(layout["cols"]):
                    x = layout["x"] + col*layout["cell_px"]
                    y = layout["y"] + row*layout["cell_px"]
                    size = layout["cell_px"]
                    points = [point(u,v) for u,v in ((x,y),(x,y+size),(x+size,y+size),(x+size,y))]
                    mesh = _quad(stage, camera_root + f"/cell_{row:02d}_{col:02d}", points, white)
                    operations.append(UsdGeom.Xformable(mesh).AddScaleOp(UsdGeom.XformOp.PrecisionFloat))
            x0, y0 = layout["x"], layout["y"]
            x1, y1 = x0+12*cell_px, y0+10*cell_px
            _quad(stage, camera_root + "/Backing",
                  [point(u,v) for u,v in ((x0,y0),(x0,y1),(x1,y1),(x1,y0))], black)
            self.handles.append({"camera_id":index,"operations":operations,"previous":None})
            self.metadata["cameras"][cam["camera_id"]] = {
                "marker_id":index,"layout":deepcopy(layout),"prim_path":camera_root,
                "roi_xyxy":[x0,y0,x1,y1],"cell_size_px":cell_px,"optical_depth_m":0.5,
            }
        self.update(WARMUP_TICK)

    def update(self, tick):
        if tick != WARMUP_TICK and not 0 <= int(tick) < WARMUP_TICK:
            raise ValueError("scene tick outside non-repeating marker domain")
        for handle in self.handles:
            cells = encode_marker(self.epoch, tick, handle["camera_id"]).reshape(-1)
            previous = handle["previous"]
            for index, (bit, operation) in enumerate(zip(cells,handle["operations"])):
                if previous is None or int(bit) != int(previous[index]):
                    operation.Set(Gf.Vec3f(0.98 if bit else 1.02))
            handle["previous"] = cells.copy()


def build_markers(stage, cameras, epoch):
    return SceneMarkers(stage, cameras, epoch)

