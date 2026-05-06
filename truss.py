bl_info = {
    "name": "Truss Analysis Master Suite",
    "author": "Pooria Danaeifar",
    "version": (3, 0),
    "blender": (5, 0, 1),
    "location": "View3D > Sidebar > Truss",
    "description": "Complete FEA workflow: Convert, Align, Connect, Intersect, Check, and Export.",
    "category": "Analysis",
}

import bpy
import bmesh
import mathutils
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, FloatProperty, BoolProperty

# ==============================================================================
#   OPERATOR 1A: STANDARD CONVERT (STRICT)
# ==============================================================================

class TRUSS_OT_ConvertLines(bpy.types.Operator):
    """Strictly converts each disjoint mesh island into its own centerline"""
    bl_idname = "truss.convert_to_lines"
    bl_label = "Convert Solid Beams"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        selected_objs = [obj for obj in context.selected_objects if obj.type == 'MESH']
        if not selected_objs:
            self.report({'WARNING'}, "No mesh objects selected.")
            return {'CANCELLED'}

        new_verts = []
        new_edges = []
        vert_index = 0
        depsgraph = context.evaluated_depsgraph_get()
        
        for obj in selected_objs:
            matrix = obj.matrix_world
            scale = matrix.to_scale()
            eval_obj = obj.evaluated_get(depsgraph)
            eval_mesh = eval_obj.to_mesh()

            bm = bmesh.new()
            bm.from_mesh(eval_mesh)
            verts_left = set(bm.verts)
            
            while verts_left:
                v = verts_left.pop()
                island = [v]
                stack = [v]
                while stack:
                    curr_v = stack.pop()
                    for edge in curr_v.link_edges:
                        other_v = edge.other_vert(curr_v)
                        if other_v in verts_left:
                            verts_left.remove(other_v)
                            stack.append(other_v)
                            island.append(other_v)
                
                if len(island) < 2: continue

                xs = [v.co.x for v in island]
                ys = [v.co.y for v in island]
                zs = [v.co.z for v in island]
                
                min_v = mathutils.Vector((min(xs), min(ys), min(zs)))
                max_v = mathutils.Vector((max(xs), max(ys), max(zs)))
                local_center = (min_v + max_v) / 2.0

                dims = [
                    (abs(max_v.x - min_v.x) * scale.x, abs(max_v.x - min_v.x), mathutils.Vector((1, 0, 0))), 
                    (abs(max_v.y - min_v.y) * scale.y, abs(max_v.y - min_v.y), mathutils.Vector((0, 1, 0))), 
                    (abs(max_v.z - min_v.z) * scale.z, abs(max_v.z - min_v.z), mathutils.Vector((0, 0, 1)))  
                ]
                world_len, local_len, local_axis = max(dims, key=lambda item: item[0])

                if world_len < 0.001: continue 

                half_vector = local_axis * (local_len / 2.0)
                new_verts.append(matrix @ (local_center - half_vector))
                new_verts.append(matrix @ (local_center + half_vector))
                new_edges.append((vert_index, vert_index + 1))
                vert_index += 2

            bm.free()
            eval_obj.to_mesh_clear()

        if not new_verts: return {'CANCELLED'}

        mesh_data = bpy.data.meshes.new("Truss_Analysis_Data")
        mesh_data.from_pydata(new_verts, new_edges, [])
        new_obj = bpy.data.objects.new("Truss_Analysis_Lines", mesh_data)
        context.collection.objects.link(new_obj)
        
        bpy.ops.object.select_all(action='DESELECT')
        new_obj.select_set(True)
        context.view_layer.objects.active = new_obj
        
        self.report({'INFO'}, f"Strict Conversion: {vert_index//2} lines.")
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 1B: ROBUST CONVERT (MERGES DISJOINT ASSEMBLIES)
# ==============================================================================

class TRUSS_OT_ConvertAssemblyLines(bpy.types.Operator):
    """Merges disjoint shapes based on proximity before creating centerlines"""
    bl_idname = "truss.convert_assembly"
    bl_label = "Convert Multi-Part Beams"
    bl_options = {'REGISTER', 'UNDO'}

    merge_threshold: bpy.props.FloatProperty(
        name="Merge Threshold (m)",
        default=0.10,
        min=0.00,
        max=2.00,
        description="Distance within which disjoint parts are merged into one element"
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "merge_threshold")

    def execute(self, context):
        selected_objs = [obj for obj in context.selected_objects if obj.type == 'MESH']
        if not selected_objs:
            self.report({'WARNING'}, "No mesh objects selected.")
            return {'CANCELLED'}

        new_verts = []
        new_edges = []
        vert_index = 0
        depsgraph = context.evaluated_depsgraph_get()
        
        for obj in selected_objs:
            matrix = obj.matrix_world
            scale = matrix.to_scale()
            eval_obj = obj.evaluated_get(depsgraph)
            eval_mesh = eval_obj.to_mesh()

            bm = bmesh.new()
            bm.from_mesh(eval_mesh)

            verts_left = set(bm.verts)
            islands = []
            while verts_left:
                v = verts_left.pop()
                island_verts = [v]
                stack = [v]
                while stack:
                    curr_v = stack.pop()
                    for edge in curr_v.link_edges:
                        other_v = edge.other_vert(curr_v)
                        if other_v in verts_left:
                            verts_left.remove(other_v)
                            stack.append(other_v)
                            island_verts.append(other_v)
                islands.append(island_verts)

            island_data = []
            for island in islands:
                if len(island) < 2: continue
                xs = [v.co.x for v in island]
                ys = [v.co.y for v in island]
                zs = [v.co.z for v in island]
                island_data.append({
                    "min": mathutils.Vector((min(xs), min(ys), min(zs))),
                    "max": mathutils.Vector((max(xs), max(ys), max(zs)))
                })

            def boxes_intersect(b1, b2, margin):
                return (b1["min"].x <= b2["max"].x + margin and b1["max"].x >= b2["min"].x - margin and
                        b1["min"].y <= b2["max"].y + margin and b1["max"].y >= b2["min"].y - margin and
                        b1["min"].z <= b2["max"].z + margin and b1["max"].z >= b2["min"].z - margin)

            merged_any = True
            while merged_any:
                merged_any = False
                new_data = []
                skip = set()
                for i in range(len(island_data)):
                    if i in skip: continue
                    current = island_data[i]
                    for j in range(i + 1, len(island_data)):
                        if j in skip: continue
                        if boxes_intersect(current, island_data[j], self.merge_threshold):
                            current["min"].x = min(current["min"].x, island_data[j]["min"].x)
                            current["min"].y = min(current["min"].y, island_data[j]["min"].y)
                            current["min"].z = min(current["min"].z, island_data[j]["min"].z)
                            current["max"].x = max(current["max"].x, island_data[j]["max"].x)
                            current["max"].y = max(current["max"].y, island_data[j]["max"].y)
                            current["max"].z = max(current["max"].z, island_data[j]["max"].z)
                            skip.add(j)
                            merged_any = True
                    new_data.append(current)
                island_data = new_data

            for data in island_data:
                min_v = data["min"]
                max_v = data["max"]
                local_center = (min_v + max_v) / 2.0

                dims = [
                    (abs(max_v.x - min_v.x) * scale.x, abs(max_v.x - min_v.x), mathutils.Vector((1, 0, 0))), 
                    (abs(max_v.y - min_v.y) * scale.y, abs(max_v.y - min_v.y), mathutils.Vector((0, 1, 0))), 
                    (abs(max_v.z - min_v.z) * scale.z, abs(max_v.z - min_v.z), mathutils.Vector((0, 0, 1)))  
                ]

                world_len, local_len, local_axis = max(dims, key=lambda item: item[0])
                if world_len < 0.001: continue 

                half_vector = local_axis * (local_len / 2.0)
                new_verts.append(matrix @ (local_center - half_vector))
                new_verts.append(matrix @ (local_center + half_vector))
                new_edges.append((vert_index, vert_index + 1))
                vert_index += 2

            bm.free()
            eval_obj.to_mesh_clear()

        if not new_verts: return {'CANCELLED'}

        mesh_data = bpy.data.meshes.new("Truss_Assembly_Lines")
        mesh_data.from_pydata(new_verts, new_edges, [])
        new_obj = bpy.data.objects.new("Truss_Analysis_Lines", mesh_data)
        context.collection.objects.link(new_obj)
        
        bpy.ops.object.select_all(action='DESELECT')
        new_obj.select_set(True)
        context.view_layer.objects.active = new_obj
        
        self.report({'INFO'}, f"Robust Conversion: {vert_index//2} lines created.")
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 2: ALIGN VERTICES (SMART LOCKING)
# ==============================================================================

class TRUSS_OT_AlignVertices(bpy.types.Operator):
    """Aligns selected vertices (and any touching vertices) to their average coordinate"""
    bl_idname = "truss.align_vertices"
    bl_label = "Align Vertices"
    bl_options = {'REGISTER', 'UNDO'}

    axis: bpy.props.StringProperty(
        name="Axis",
        description="Axis to align (X, Y, Z, XY, XZ, YZ)",
        default="Z"
    )

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'WARNING'}, "Must be in Edit Mode.")
            return {'CANCELLED'}

        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        
        sel_verts = [v for v in bm.verts if v.select]

        tol = getattr(context.scene, "align_tolerance", 0.005)

        # If all verts are selected, cluster by axis within tolerance
        all_selected = (len(sel_verts) == len(bm.verts))

        if len(sel_verts) < 2:
            self.report({'WARNING'}, "Select at least 2 vertices to align.")
            return {'CANCELLED'}

        # Group-and-align when user selected all verts: cluster verts whose
        # coordinate(s) are within the tolerance and align each cluster separately.
        if all_selected:
            groups = []
            for v in bm.verts:
                placed = False
                for g in groups:
                    rep = g[0]
                    ok = True
                    if 'X' in self.axis and abs(v.co.x - rep.co.x) > tol: ok = False
                    if 'Y' in self.axis and abs(v.co.y - rep.co.y) > tol: ok = False
                    if 'Z' in self.axis and abs(v.co.z - rep.co.z) > tol: ok = False
                    if ok:
                        g.append(v)
                        placed = True
                        break
                if not placed:
                    groups.append([v])

            moved = 0
            for g in groups:
                if len(g) < 2: continue
                if 'X' in self.axis:
                    avg_x = sum(v.co.x for v in g) / len(g)
                if 'Y' in self.axis:
                    avg_y = sum(v.co.y for v in g) / len(g)
                if 'Z' in self.axis:
                    avg_z = sum(v.co.z for v in g) / len(g)

                for v in g:
                    if 'X' in self.axis: v.co.x = avg_x
                    if 'Y' in self.axis: v.co.y = avg_y
                    if 'Z' in self.axis: v.co.z = avg_z
                    moved += 1

            bmesh.update_edit_mesh(obj.data)
            self.report({'INFO'}, f"Aligned {moved} vertices in {len([g for g in groups if len(g)>1])} clusters using tolerance {tol}.")
            return {'FINISHED'}

        # Otherwise fall back to original behaviour but use the tolerance
        avg_x = sum(v.co.x for v in sel_verts) / len(sel_verts)
        avg_y = sum(v.co.y for v in sel_verts) / len(sel_verts)
        avg_z = sum(v.co.z for v in sel_verts) / len(sel_verts)

        verts_to_move = set(sel_verts)
        for v in bm.verts:
            if not v.select:
                for sv in sel_verts:
                    if (v.co - sv.co).length < tol:
                        verts_to_move.add(v)
                        break

        for v in verts_to_move:
            if 'X' in self.axis: v.co.x = avg_x
            if 'Y' in self.axis: v.co.y = avg_y
            if 'Z' in self.axis: v.co.z = avg_z

        bmesh.update_edit_mesh(obj.data)
        self.report({'INFO'}, f"Aligned {len(verts_to_move)} grouped vertices on {self.axis} axis (tol={tol}).")
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 3: CONNECT & EXTEND LINES (FIX GAPS)
# ==============================================================================

class TRUSS_OT_ConnectLines(bpy.types.Operator):
    """Extends loose line ends straight to the nearest structural beam"""
    bl_idname = "truss.connect_lines"
    bl_label = "Connect/Extend Lines"
    bl_options = {'REGISTER', 'UNDO'}
    
    max_extension: bpy.props.FloatProperty(
        name="Max Extension (m)",
        default=0.5,
        min=0.01,
        max=5.0,
        description="Maximum distance a line will extend to find a target."
    )

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'WARNING'}, "Please select the truss line object.")
            return {'CANCELLED'}

        was_in_edit_mode = (context.mode == 'EDIT_MESH')
        if not was_in_edit_mode:
            bpy.ops.object.mode_set(mode='EDIT')

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        target_edges = []
        for e in bm.edges:
            p1 = obj.matrix_world @ e.verts[0].co
            p2 = obj.matrix_world @ e.verts[1].co
            target_edges.append((p1, p2, e))

        has_selection = any(v.select for v in bm.verts)

        if was_in_edit_mode and has_selection:
            loose_tips = [v for v in bm.verts if len(v.link_edges) == 1 and v.select]
            mode_msg = "selected"
        else:
            loose_tips = [v for v in bm.verts if len(v.link_edges) == 1]
            mode_msg = "all"

        if not loose_tips:
            if not was_in_edit_mode: bpy.ops.object.mode_set(mode='OBJECT')
            return {'CANCELLED'}

        verts_to_move = {}
        extended_count = 0

        for tip in loose_tips:
            tip_co_world = obj.matrix_world @ tip.co
            edge = tip.link_edges[0]
            other_v = edge.other_vert(tip)
            other_v_world = obj.matrix_world @ other_v.co
            
            direction = (tip_co_world - other_v_world).normalized()
            best_hit_location = None
            min_dist = self.max_extension
            
            for (p1, p2, target_edge) in target_edges:
                if target_edge == edge: continue
                
                line_a_p1 = tip_co_world
                line_a_p2 = tip_co_world + (direction * self.max_extension)
                
                res = mathutils.geometry.intersect_line_line(line_a_p1, line_a_p2, p1, p2)
                if res is None: continue 
                
                pa, pb = res
                
                len_segment = (p2 - p1).length
                dist_p1 = (pb - p1).length
                dist_p2 = (pb - p2).length
                
                if dist_p1 <= len_segment + 0.01 and dist_p2 <= len_segment + 0.01:
                    vec_to_hit = pa - tip_co_world
                    if vec_to_hit.dot(direction) > 0: 
                        hit_dist = vec_to_hit.length
                        gap = (pa - pb).length
                        if hit_dist < min_dist and gap < 0.1:
                            min_dist = hit_dist
                            best_hit_location = pb

            if best_hit_location:
                local_pos = obj.matrix_world.inverted() @ best_hit_location
                verts_to_move[tip] = local_pos
                extended_count += 1

        for v, pos in verts_to_move.items():
            v.co = pos
            
        bmesh.update_edit_mesh(obj.data)
        
        if not was_in_edit_mode:
            bpy.ops.object.mode_set(mode='OBJECT')
        
        self.report({'INFO'}, f"Extended and connected {extended_count} {mode_msg} loose ends.")
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 4: REBUILD & WELD (ROBUST INTERSECTION FIX)
# ==============================================================================

class TRUSS_OT_RebuildIntersections(bpy.types.Operator):
    """Calculates intersections, deletes old lines, and rebuilds connected segments"""
    bl_idname = "truss.rebuild_intersections"
    bl_label = "Rebuild Intersections"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.active_object.mode != 'EDIT':
            self.report({'WARNING'}, "Please enter Edit Mode and select lines.")
            return {'CANCELLED'}

        obj = context.edit_object
        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()

        edges_to_process = []
        for e in bm.edges:
            if e.select:
                edges_to_process.append({
                    "edge": e,
                    "v1": e.verts[0],
                    "v2": e.verts[1],
                    "p1": e.verts[0].co.copy(),
                    "p2": e.verts[1].co.copy(),
                    "split_points": [] 
                })

        if len(edges_to_process) < 2: return {'CANCELLED'}

        count = 0
        for i in range(len(edges_to_process)):
            for j in range(i + 1, len(edges_to_process)):
                item1 = edges_to_process[i]
                item2 = edges_to_process[j]
                
                res = mathutils.geometry.intersect_line_line(item1["p1"], item1["p2"], item2["p1"], item2["p2"])
                if res is None: continue
                
                p_on_1, p_on_2 = res
                if (p_on_1 - p_on_2).length > 0.05: continue 
                
                mid_point = (p_on_1 + p_on_2) / 2
                
                len1 = (item1["p1"] - item1["p2"]).length
                dist1 = (mid_point - item1["p1"]).length + (mid_point - item1["p2"]).length
                
                len2 = (item2["p1"] - item2["p2"]).length
                dist2 = (mid_point - item2["p1"]).length + (mid_point - item2["p2"]).length
                
                if dist1 <= len1 + 0.001 and dist2 <= len2 + 0.001:
                    d_start1 = (mid_point - item1["p1"]).length
                    d_end1 = (mid_point - item1["p2"]).length
                    if d_start1 > 0.01 and d_end1 > 0.01:
                        item1["split_points"].append(mid_point)
                        
                    d_start2 = (mid_point - item2["p1"]).length
                    d_end2 = (mid_point - item2["p2"]).length
                    if d_start2 > 0.01 and d_end2 > 0.01:
                        item2["split_points"].append(mid_point)
                        
                    count += 1

        if count == 0: return {'CANCELLED'}

        edges_to_delete = []
        for item in edges_to_process:
            splits = item["split_points"]
            if not splits: continue
            
            start_co = item["p1"]
            splits.sort(key=lambda p: (p - start_co).length)
            chain_coords = [start_co] + splits + [item["p2"]]
            prev_vert = item["v1"]
            
            for k in range(1, len(chain_coords)):
                coord = chain_coords[k]
                target_vert = item["v2"] if k == len(chain_coords) - 1 else bm.verts.new(coord)
                try:
                    bm.edges.new((prev_vert, target_vert))
                except ValueError: pass 
                prev_vert = target_vert
            
            edges_to_delete.append(item["edge"])

        bmesh.ops.delete(bm, geom=edges_to_delete, context='EDGES')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.remove_doubles(threshold=0.001)
        
        bmesh.update_edit_mesh(me)
        self.report({'INFO'}, f"Success! Rebuilt truss with {count} intersections.")
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 5: SANITY CHECK (DIAGNOSTICS)
# ==============================================================================

class TRUSS_OT_SanityCheck(bpy.types.Operator):
    """Highlights problem geometry (micro-edges and floating vertices)"""
    bl_idname = "truss.sanity_check"
    bl_label = "Run Sanity Check"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'WARNING'}, "Must be in Edit Mode.")
            return {'CANCELLED'}

        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        
        # Deselect everything first
        bpy.ops.mesh.select_all(action='DESELECT')
        
        issues_found = 0

        # Check 1: Find Micro-Edges (shorter than 1mm)
        for e in bm.edges:
            if e.calc_length() < 0.001:
                e.select = True
                e.verts[0].select = True
                e.verts[1].select = True
                issues_found += 1

        # Check 2: Find completely floating vertices
        for v in bm.verts:
            if len(v.link_edges) == 0:
                v.select = True
                issues_found += 1

        bmesh.update_edit_mesh(obj.data)

        if issues_found > 0:
            self.report({'WARNING'}, f"Found {issues_found} issues! Problematic geometry has been selected.")
        else:
            self.report({'INFO'}, "Passed! Truss geometry is clean and FEA ready.")
            
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 6: WEIGHT CALCULATOR
# ==============================================================================

class TRUSS_OT_CalculateWeight(bpy.types.Operator):
    """Calculates object weight from mesh volume or manual area and thickness"""
    bl_idname = "truss.calculate_weight"
    bl_label = "Calculate Weight"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        density = context.scene.calc_density
        thickness_manual = context.scene.calc_thickness
        is_double_sided = context.scene.calc_double_sided
        force_override = context.scene.calc_force_override

        selected_objs = [obj for obj in context.selected_objects if obj.type == 'MESH']
        if not selected_objs:
            self.report({'WARNING'}, "No mesh objects selected.")
            return {'CANCELLED'}

        total_volume = 0.0
        method_report = "Volume (Geometric)"
        depsgraph = context.evaluated_depsgraph_get()

        for obj in selected_objs:
            obj_eval = obj.evaluated_get(depsgraph)
            mesh = obj_eval.to_mesh()

            bm = bmesh.new()
            bm.from_mesh(mesh)
            bmesh.ops.transform(bm, matrix=obj.matrix_world, verts=bm.verts)

            use_manual = force_override
            vol = 0.0

            if not use_manual:
                vol = bm.calc_volume()
                if abs(vol) < 0.0001:
                    use_manual = True

            if use_manual:
                area = sum(face.calc_area() for face in bm.faces)
                if is_double_sided:
                    area = area / 2.0
                vol = area * thickness_manual
                method_report = "Manual (Area x Thickness)"

            total_volume += abs(vol)
            bm.free()
            obj_eval.to_mesh_clear()

        weight_kg = total_volume * density
        weight_kn = (weight_kg * 9.81) / 1000.0

        context.scene["last_calc_volume"] = total_volume
        context.scene["last_calc_weight"] = weight_kg
        context.scene["last_calc_force"] = weight_kn
        context.scene["last_calc_method"] = method_report
        context.scene["last_obj_count"] = len(selected_objs)

        self.report({'INFO'}, f"Calculated weight for {len(selected_objs)} objects.")
        return {'FINISHED'}


# ==============================================================================
#   OPERATOR 7: EXPORT TO DXF
# ==============================================================================

class TRUSS_OT_ExportDXF(bpy.types.Operator, ExportHelper):
    """Exports the truss lines directly to a clean 3D DXF file"""
    bl_idname = "truss.export_dxf"
    bl_label = "Export DXF"
    
    filename_ext = ".dxf"
    filter_glob: StringProperty(
        default="*.dxf",
        options={'HIDDEN'},
        maxlen=255, 
    )

    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'WARNING'}, "Please select the truss line object.")
            return {'CANCELLED'}
        
        was_in_edit_mode = (context.mode == 'EDIT_MESH')
        if was_in_edit_mode:
            bpy.ops.object.mode_set(mode='OBJECT')
            
        mesh = obj.data
        mat = obj.matrix_world
        
        try:
            with open(self.filepath, 'w') as f:
                # Write minimal DXF Header for Lines
                f.write("0\nSECTION\n2\nENTITIES\n")
                
                # Write all edges
                for edge in mesh.edges:
                    p1 = mat @ mesh.vertices[edge.vertices[0]].co
                    p2 = mat @ mesh.vertices[edge.vertices[1]].co
                    
                    f.write("0\nLINE\n8\nTruss_Lines\n")
                    f.write(f"10\n{p1.x}\n20\n{p1.y}\n30\n{p1.z}\n")
                    f.write(f"11\n{p2.x}\n21\n{p2.y}\n31\n{p2.z}\n")
                    
                f.write("0\nENDSEC\n0\nEOF\n")
                
            self.report({'INFO'}, f"Successfully exported to: {self.filepath}")
            
        except Exception as e:
            self.report({'ERROR'}, f"Failed to export: {str(e)}")
            
        if was_in_edit_mode:
            bpy.ops.object.mode_set(mode='EDIT')
            
        return {'FINISHED'}


# ==============================================================================
#   UI PANEL
# ==============================================================================

class TRUSS_PT_MainPanel(bpy.types.Panel):
    """Creates a Panel in the View3D UI Sidebar"""
    bl_label = "Truss Master Tools"
    bl_idname = "TRUSS_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Truss"

    def draw(self, context):
        layout = self.layout
        is_edit = (context.mode == 'EDIT_MESH')

        # --- STEP 0: WEIGHT CALCULATOR ---
        col = layout.column(align=True)
        col.label(text="Step 0: Weight Calculator", icon='MOD_SIMPLEDEFORM')
        col.prop(context.scene, "calc_density", text="Density (kg/m³)")
        col.prop(context.scene, "calc_force_override", text="Force Manual Thickness")

        if context.scene.calc_force_override:
            box = col.box()
            box.label(text="Geometry volume is ignored.", icon='ERROR')
            box.prop(context.scene, "calc_thickness", text="Manual Thickness (m)")
            box.prop(context.scene, "calc_double_sided", text="Double Sided Geometry")
        else:
            col.label(text="Uses geometric volume when available.", icon='INFO')

        col.operator("truss.calculate_weight", text="Calculate Weight", icon='PHYSICS')

        if context.scene.get("last_calc_weight") is not None:
            box = col.box()
            box.label(text=f"Selected Objects: {context.scene.get('last_obj_count', 0)}")
            box.label(text=f"Total Weight: {context.scene['last_calc_weight']:.2f} kg")
            row = box.row()
            row.alert = True
            row.label(text=f"Load: {context.scene['last_calc_force']:.2f} kN")
            box.label(text=f"Method: {context.scene.get('last_calc_method', 'Unknown')}")
        col.separator()
        
        # --- HELPER BUTTON ---
        if not is_edit:
            layout.operator("object.editmode_toggle", text="Enter Edit Mode for Geometry Tools", icon='EDITMODE_HLT')
            layout.separator()

        # --- STEP 1: IMPORT ---
        col = layout.column(align=True)
        col.label(text="Step 1: Import Geometry", icon='IMPORT')
        col.operator("truss.convert_to_lines", text="Convert Solid Beams", icon='CURVE_DATA')
        col.operator("truss.convert_assembly", text="Convert Multi-Part Beams", icon='GROUP')
        col.separator()
        
        # --- STEP 2: ALIGN ---
        col = layout.column(align=True)
        col.label(text="Step 2: Align Nodes", icon='SNAP_VERTEX')
        
        align_col = col.column(align=True)
        align_col.active = is_edit
        align_col.prop(context.scene, "align_tolerance", text="Tolerance (m)")
        
        row = align_col.row(align=True)
        row.operator("truss.align_vertices", text="Align X").axis = 'X'
        row.operator("truss.align_vertices", text="Align Y").axis = 'Y'
        row.operator("truss.align_vertices", text="Align Z").axis = 'Z'
        
        row2 = align_col.row(align=True)
        row2.operator("truss.align_vertices", text="Align YZ").axis = 'YZ'
        row2.operator("truss.align_vertices", text="Align XZ").axis = 'XZ'
        row2.operator("truss.align_vertices", text="Align XY").axis = 'XY'
        col.separator()

        # --- STEP 3: CONNECT ---
        col = layout.column(align=True)
        col.label(text="Step 3: Clean Gaps", icon='SNAP_ON')
        col.operator("truss.connect_lines", text="Extend & Connect", icon='DRIVER')
        col.separator()
        
        # --- STEP 4: INTERSECTIONS ---
        col = layout.column(align=True)
        col.label(text="Step 4: Structural Nodes", icon='VERTEXSEL')
        
        edit_col = col.column(align=True)
        edit_col.active = is_edit
        edit_col.operator("truss.rebuild_intersections", text="Rebuild Intersections", icon='MOD_BOOLEAN')
        edit_col.label(text="(Select ALL lines first)", icon='INFO')
        col.separator()
            
        # --- STEP 5: SANITY CHECK ---
        col = layout.column(align=True)
        col.label(text="Step 5: Sanity Check", icon='VIEWZOOM')
        
        check_col = col.column(align=True)
        check_col.active = is_edit
        check_col.operator("truss.sanity_check", text="Run Diagnostics", icon='CHECKMARK')
        check_col.label(text="(Highlights invalid geometry)", icon='INFO')
        col.separator()
        
        # --- STEP 6: EXPORT ---
        col = layout.column(align=True)
        col.label(text="Step 6: Export", icon='EXPORT')
        col.operator("truss.export_dxf", text="Export to DXF", icon='FILE_3D')


# ==============================================================================
#   REGISTRATION
# ==============================================================================

classes = (
    TRUSS_OT_CalculateWeight,
    TRUSS_OT_ConvertLines,
    TRUSS_OT_ConvertAssemblyLines,
    TRUSS_OT_AlignVertices,
    TRUSS_OT_ConnectLines,
    TRUSS_OT_RebuildIntersections,
    TRUSS_OT_SanityCheck,
    TRUSS_OT_ExportDXF,
    TRUSS_PT_MainPanel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.calc_density = FloatProperty(name="Density", default=2700.0)
    bpy.types.Scene.calc_thickness = FloatProperty(name="Thickness", default=0.03)
    bpy.types.Scene.calc_double_sided = BoolProperty(name="Double Sided", default=True)
    bpy.types.Scene.calc_force_override = BoolProperty(name="Force Override", default=False)
    bpy.types.Scene.align_tolerance = FloatProperty(name="Align Tolerance", default=0.005, min=0.0, description="Distance within which vertices are grouped for alignment")

def unregister():
    del bpy.types.Scene.calc_density
    del bpy.types.Scene.calc_thickness
    del bpy.types.Scene.calc_double_sided
    del bpy.types.Scene.calc_force_override
    del bpy.types.Scene.align_tolerance
    for cls in classes:
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()