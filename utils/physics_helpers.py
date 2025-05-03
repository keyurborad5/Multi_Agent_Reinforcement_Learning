import pybullet as p
import pybullet_data
import numpy as np
import time
import math

def create_wall(x_center, y_center, half_ext_x, half_ext_y, height=0.4):
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_ext_x, half_ext_y, height / 2.0]
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_ext_x, half_ext_y, height / 2.0],
        rgbaColor=[0.8, 0.8, 0.8, 1]
    )
    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x_center, y_center, height / 2.0]
    )
    return body_id

def create_cube_block(pos_x, pos_y, size=0.2, mass=100.00):
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size/2, size/2, size/2]
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size/2, size/2, size/2],
        rgbaColor=[0, 1, 0, 1]  # green
    )
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos_x, pos_y, size/2]
    )
    return body_id

def create_point_agent(pos_x, pos_y, radius=0.05, mass=1.0, color=[1, 0, 0, 1]):
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos_x, pos_y, radius]
    )
    return body_id

def distance_between_bodies(bodyA, bodyB):
    """
    Returns Euclidean distance between the base positions of bodyA and bodyB.
    """
    posA, _ = p.getBasePositionAndOrientation(bodyA)
    posB, _ = p.getBasePositionAndOrientation(bodyB)
    dx = posA[0] - posB[0]
    dy = posA[1] - posB[1]
    dz = posA[2] - posB[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)
# ─────────────────────────────────────────────────────────────────────────────
# Helper: convert a world-space point to a body-local point
# ─────────────────────────────────────────────────────────────────────────────
def world_to_local(body_id, world_pt):
    base_pos, base_orn = p.getBasePositionAndOrientation(body_id)
    inv_pos, inv_orn   = p.invertTransform(base_pos, base_orn)
    local_pt, _        = p.multiplyTransforms(inv_pos, inv_orn,
                                              world_pt, [0, 0, 0, 1])
    return local_pt
# ─────────────────────────────────────────────────────────────────────────────
# Helper: project "agent → cube-centre" line onto the cube surface
#         so we grab the *wall* not the centre.
# ─────────────────────────────────────────────────────────────────────────────
def closest_point_on_cube_surface(cube_id, query_pt):
    # get cube AABB in world coords
    aabb_min, aabb_max = p.getAABB(cube_id)

    # clamp query point to AABB to get the closest point *inside* the cube
    cp_inside = np.clip(query_pt, aabb_min, aabb_max)

    # now push that point to the nearest face, i.e. choose the axis on which
    # |difference| to the face is smallest and project to the face
    dists_to_min = np.abs(cp_inside - aabb_min)
    dists_to_max = np.abs(aabb_max - cp_inside)
    axis          = np.argmin(np.minimum(dists_to_min, dists_to_max))

    # move to the cube surface along that axis
    if cp_inside[axis] - aabb_min[axis] < aabb_max[axis] - cp_inside[axis]:
        cp_surface = cp_inside.copy()
        cp_surface[axis] = aabb_min[axis]      # snap to − face
    else:
        cp_surface = cp_inside.copy()
        cp_surface[axis] = aabb_max[axis]      # snap to + face
    return cp_surface.tolist()


# ─────────────────────────────────────────────────────────────────────────────
# The new constraint creator
# ─────────────────────────────────────────────────────────────────────────────
def create_fixed_constraint(bodyA, bodyB, grab_distance_threshold=0.25):
    """
    bodyA … the *agent*  (parent)
    bodyB … the *cube*   (child)

    Creates a rigid joint between the agent surface and the nearest cube wall.
    Returns the constraint id or None if the agent is too far away.
    """
    # 1️⃣ work in world space ---------------------------------------------------
    agent_pos, _ = p.getBasePositionAndOrientation(bodyA)
    cube_pos,  _ = p.getBasePositionAndOrientation(bodyB)

    # bail out if they are too far apart
    if np.linalg.norm(np.array(agent_pos) - np.array(cube_pos)) > grab_distance_threshold:
        return None

    # 2️⃣ pick the cube-wall point --------------------------------------------
    pivot_world = closest_point_on_cube_surface(bodyB, agent_pos)

    # 3️⃣ compute local frames for each body -----------------------------------
    parent_local = world_to_local(bodyA, pivot_world)
    child_local  = world_to_local(bodyB, pivot_world)

    # 4️⃣ make the joint --------------------------------------------------------
    cid = p.createConstraint(parentBodyUniqueId=bodyA, parentLinkIndex=-1,
                             childBodyUniqueId=bodyB,  childLinkIndex=-1,
                             jointType=p.JOINT_FIXED,  jointAxis=[0, 0, 0],
                             parentFramePosition=parent_local,
                             childFramePosition=child_local)

    # give it plenty of authority
    p.changeConstraint(cid, maxForce=1e6)
    return cid
