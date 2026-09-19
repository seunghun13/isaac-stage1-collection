"""Rigid TrackingCenter child board with fixed border and translated code cells.

Board axes are x right, y down, z away from its intended observer. The caller
sets the constant mounting matrix once; this module never faces a camera again.
"""
from copy import deepcopy

from pxr import Gf, Sdf, UsdGeom, UsdShade

from marker_codec import WARMUP_TICK, encode_marker, encoding_metadata


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


def _quad(stage, path, xyxy, depth, material):
    x0,y0,x1,y1 = xyxy
    # Front faces must point toward -board-z. Scout01 used +z winding and
    # rendered even its white quiet plate black despite valid white binding.
    # Correcting winding tests that hypothesis; it does not change the decoder.
    points = [Gf.Vec3f(x,y,depth) for x,y in ((x0,y0),(x0,y1),(x1,y1),(x1,y0))]
    mesh = UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4]); mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none); mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr([Gf.Vec3f(x0,y0,depth),Gf.Vec3f(x1,y1,depth)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh


class DroneBoard:
    def __init__(self,stage,config,epoch):
        self.stage = stage
        self.epoch = int(epoch)
        self.board_id = int(config["board_id"])
        if not 101 <= self.board_id <= 199:
            raise ValueError("drone board IDs must be 101..199, distinct from screen IDs")
        cell = float(config["cell_size_m"])
        if not 0 < cell < 1.0:
            raise ValueError("cell_size_m must be positive and below 1m")
        epsilon = float(config.get("cell_depth_offset_m",min(0.002,cell*0.05)))
        if not 0 < epsilon < cell*0.25:
            raise ValueError("cell depth must be small relative to cell width")
        parent = config.get("tracking_center_prim_path","/World/drone_1/base_link/TrackingCenter")
        if not stage.GetPrimAtPath(parent):
            raise ValueError("TrackingCenter parent missing")
        self.path = parent + f"/ValidationTimeBoard_{self.board_id}"
        self.xform = UsdGeom.Xform.Define(stage,self.path)
        matrix = config["T_tracking_center_board_row"]
        self.xform.AddTransformOp().Set(Gf.Matrix4d(*[v for row in matrix for v in row]))
        white = _material(stage,self.path+"/WhiteMaterial",0.85)
        black = _material(stage,self.path+"/BlackMaterial",0.002)
        # One white outer quiet plate, a fixed black ring, a fixed white inset,
        # and a fixed black code backing. Only the separate white cells move.
        _quad(stage,self.path+"/QuietPlate",[-9*cell,-8*cell,9*cell,8*cell],3*epsilon,white)
        reference_z = epsilon
        ring = [(-8,-7,8,-6),(-8,6,8,7),(-8,-6,-7,6),(7,-6,8,6)]
        for i,rect in enumerate(ring):
            _quad(stage,self.path+f"/FixedRing_{i}",[v*cell for v in rect],reference_z,black)
        _quad(stage,self.path+"/WhiteInset",[-7*cell,-6*cell,7*cell,6*cell],2*epsilon,white)
        _quad(stage,self.path+"/CodeBacking",[-6*cell,-5*cell,6*cell,5*cell],0,black)
        self.operations = []
        for row in range(10):
            for col in range(12):
                x0,y0 = (col-6)*cell,(row-5)*cell
                mesh = _quad(stage,self.path+f"/Cell_{row:02d}_{col:02d}",
                             [x0,y0,x0+cell,y0+cell],0,white)
                self.operations.append(UsdGeom.Xformable(mesh).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble))
        self.previous = None
        self.epsilon = epsilon
        self.tick = None
        corners = [[-8*cell,-7*cell,reference_z],[8*cell,-7*cell,reference_z],
                   [8*cell,7*cell,reference_z],[-8*cell,7*cell,reference_z]]
        self.metadata = {
            "schema":"mro_drone_board_v1","enabled":True,"role":"drone_rigid_time_and_fixed_geometry",
            "board_id":self.board_id,"codec_camera_id_field":self.board_id,"epoch":self.epoch,
            "target_camera":config["target_camera"],"prim_path":self.path,
            "parent_prim_path":parent,"parent_frame":"drone_1/tracking_center",
            "T_tracking_center_board_row":deepcopy(matrix),
            "cell_size_m":cell,"cell_depth_offset_m":epsilon,
            "board_axes":"x right, y down, z away from intended observer; fixed rigid mount",
            "white_cell_depths_board_m":{"bit_1":-epsilon,"bit_0":epsilon},
            "fixed_reference_points_board_m":corners,
            "fixed_reference_point_order":["top_left","top_right","bottom_right","bottom_left"],
            "reference_geometry":"outer edges of fixed black ring; excludes movable code cells",
            "reference_plane_z_board_m":reference_z,
            "outer_ring_size_cells":[16,14],"outer_quiet_plate_size_cells":[18,16],
            "code_bounds_relative_outer_ring_cells":[2,2,14,12],
            "search_roi_xyxy":list(config["search_roi_xyxy"]),
            "vision_layout":{
                "outer_ring_cells":[16,14],"canonical_cell_px":12,
                "payload_layout":{"x":24,"y":24,"cell_px":12,"cols":12,"rows":10},
                "minimum_side_px":48,"minimum_area_px":3000,
                "minimum_reference_contrast":60,"maximum_edge_fit_rms_px":0.8,
                "board_id":self.board_id,
            },
            "pixel_coordinates":"pixel centers at (column+0.5,row+0.5); fixed geometric edge corners",
            "encoding":encoding_metadata(),
            "update":"changed white-cell local normal translation only; fixed border/backing/materials/mount",
            "front_face_winding":"TL,BL,BR,TR; geometric normal -board-z toward intended observer",
        }
        self.update(WARMUP_TICK)

    def update(self,tick):
        tick = int(tick)
        if not 0 <= tick <= WARMUP_TICK:
            raise ValueError("tick must be nonnegative uint16; 65535 reserved for warmup")
        cells = encode_marker(self.epoch,tick,self.board_id).reshape(-1)
        for i,(bit,operation) in enumerate(zip(cells,self.operations)):
            if self.previous is None or int(bit)!=int(self.previous[i]):
                operation.Set(Gf.Vec3d(0,0,-self.epsilon if bit else self.epsilon))
        self.previous = cells.copy(); self.tick = tick

    def readback(self):
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(self.xform.GetPrim())
        return {
            "board_id":self.board_id,"scene_tick_applied":self.tick,"prim_path":self.path,
            "T_world_board_row":[list(row) for row in matrix],
            "world_corners_m":[list(matrix.Transform(Gf.Vec3d(*p))) for p in self.metadata["fixed_reference_points_board_m"]],
            "source":"actual USD XformCache readback of fixed board and fixed reference vertices",
        }


def build_drone_board(stage,config,epoch):
    return DroneBoard(stage,config,epoch)
