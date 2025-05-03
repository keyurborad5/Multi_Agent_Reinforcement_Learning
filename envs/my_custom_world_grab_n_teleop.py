import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import sys
import os

# Get parent directory of "project/"
project_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(project_dir)

# Add it to Python path so it can find 'utils'
sys.path.append(parent_dir)
from utils.physics_helpers import (create_wall, create_cube_block, 
                                   create_point_agent,create_fixed_constraint, 
                                   distance_between_bodies, world_to_local,
                                   closest_point_on_cube_surface)

# def create_wall(x_center, y_center, half_ext_x, half_ext_y, height=0.4):
#     collision_shape = p.createCollisionShape(
#         shapeType=p.GEOM_BOX,
#         halfExtents=[half_ext_x, half_ext_y, height / 2.0]
#     )
#     visual_shape = p.createVisualShape(
#         shapeType=p.GEOM_BOX,
#         halfExtents=[half_ext_x, half_ext_y, height / 2.0],
#         rgbaColor=[0.8, 0.8, 0.8, 1]
#     )
#     body_id = p.createMultiBody(
#         baseMass=0,
#         baseCollisionShapeIndex=collision_shape,
#         baseVisualShapeIndex=visual_shape,
#         basePosition=[x_center, y_center, height / 2.0]
#     )
#     return body_id

# def create_cube_block(pos_x, pos_y, size=0.2, mass=100.00):
#     collision_shape = p.createCollisionShape(
#         shapeType=p.GEOM_BOX,
#         halfExtents=[size/2, size/2, size/2]
#     )
#     visual_shape = p.createVisualShape(
#         shapeType=p.GEOM_BOX,
#         halfExtents=[size/2, size/2, size/2],
#         rgbaColor=[0, 1, 0, 1]  # green
#     )
#     body_id = p.createMultiBody(
#         baseMass=mass,
#         baseCollisionShapeIndex=collision_shape,
#         baseVisualShapeIndex=visual_shape,
#         basePosition=[pos_x, pos_y, size/2]
#     )
#     return body_id

# def create_point_agent(pos_x, pos_y, radius=0.05, mass=1.0, color=[1, 0, 0, 1]):
#     collision_shape = p.createCollisionShape(
#         shapeType=p.GEOM_SPHERE,
#         radius=radius
#     )
#     visual_shape = p.createVisualShape(
#         shapeType=p.GEOM_SPHERE,
#         radius=radius,
#         rgbaColor=color
#     )
#     body_id = p.createMultiBody(
#         baseMass=mass,
#         baseCollisionShapeIndex=collision_shape,
#         baseVisualShapeIndex=visual_shape,
#         basePosition=[pos_x, pos_y, radius]
#     )
#     return body_id

# def distance_between_bodies(bodyA, bodyB):
#     """
#     Returns Euclidean distance between the base positions of bodyA and bodyB.
#     """
#     posA, _ = p.getBasePositionAndOrientation(bodyA)
#     posB, _ = p.getBasePositionAndOrientation(bodyB)
#     dx = posA[0] - posB[0]
#     dy = posA[1] - posB[1]
#     dz = posA[2] - posB[2]
#     return math.sqrt(dx*dx + dy*dy + dz*dz)
# # ─────────────────────────────────────────────────────────────────────────────
# # Helper: convert a world-space point to a body-local point
# # ─────────────────────────────────────────────────────────────────────────────
# def world_to_local(body_id, world_pt):
#     base_pos, base_orn = p.getBasePositionAndOrientation(body_id)
#     inv_pos, inv_orn   = p.invertTransform(base_pos, base_orn)
#     local_pt, _        = p.multiplyTransforms(inv_pos, inv_orn,
#                                               world_pt, [0, 0, 0, 1])
#     return local_pt
# # ─────────────────────────────────────────────────────────────────────────────
# # Helper: project "agent → cube-centre" line onto the cube surface
# #         so we grab the *wall* not the centre.
# # ─────────────────────────────────────────────────────────────────────────────
# def closest_point_on_cube_surface(cube_id, query_pt):
#     # get cube AABB in world coords
#     aabb_min, aabb_max = p.getAABB(cube_id)

#     # clamp query point to AABB to get the closest point *inside* the cube
#     cp_inside = np.clip(query_pt, aabb_min, aabb_max)

#     # now push that point to the nearest face, i.e. choose the axis on which
#     # |difference| to the face is smallest and project to the face
#     dists_to_min = np.abs(cp_inside - aabb_min)
#     dists_to_max = np.abs(aabb_max - cp_inside)
#     axis          = np.argmin(np.minimum(dists_to_min, dists_to_max))

#     # move to the cube surface along that axis
#     if cp_inside[axis] - aabb_min[axis] < aabb_max[axis] - cp_inside[axis]:
#         cp_surface = cp_inside.copy()
#         cp_surface[axis] = aabb_min[axis]      # snap to − face
#     else:
#         cp_surface = cp_inside.copy()
#         cp_surface[axis] = aabb_max[axis]      # snap to + face
#     return cp_surface.tolist()


# # ─────────────────────────────────────────────────────────────────────────────
# # The new constraint creator
# # ─────────────────────────────────────────────────────────────────────────────
# def create_fixed_constraint(bodyA, bodyB, grab_distance_threshold=0.25):
#     """
#     bodyA … the *agent*  (parent)
#     bodyB … the *cube*   (child)

#     Creates a rigid joint between the agent surface and the nearest cube wall.
#     Returns the constraint id or None if the agent is too far away.
#     """
#     # 1️⃣ work in world space ---------------------------------------------------
#     agent_pos, _ = p.getBasePositionAndOrientation(bodyA)
#     cube_pos,  _ = p.getBasePositionAndOrientation(bodyB)

#     # bail out if they are too far apart
#     if np.linalg.norm(np.array(agent_pos) - np.array(cube_pos)) > grab_distance_threshold:
#         return None

#     # 2️⃣ pick the cube-wall point --------------------------------------------
#     pivot_world = closest_point_on_cube_surface(bodyB, agent_pos)

#     # 3️⃣ compute local frames for each body -----------------------------------
#     parent_local = world_to_local(bodyA, pivot_world)
#     child_local  = world_to_local(bodyB, pivot_world)

#     # 4️⃣ make the joint --------------------------------------------------------
#     cid = p.createConstraint(parentBodyUniqueId=bodyA, parentLinkIndex=-1,
#                              childBodyUniqueId=bodyB,  childLinkIndex=-1,
#                              jointType=p.JOINT_FIXED,  jointAxis=[0, 0, 0],
#                              parentFramePosition=parent_local,
#                              childFramePosition=child_local)

#     # give it plenty of authority
#     p.changeConstraint(cid, maxForce=1e6)
#     return cid

def main():
    # Connect to PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Create a plane (floor)
    plane_id = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    # Create outer boundary walls (6x6 area, from -3 to +3 in x,y)
    create_wall(x_center=0,    y_center= 3.0, half_ext_x=3.0, half_ext_y=0.05)  # Top
    create_wall(x_center=0,    y_center=-3.0, half_ext_x=3.0, half_ext_y=0.05)  # Bottom
    create_wall(x_center=-3.0, y_center= 0.0, half_ext_x=0.05, half_ext_y=3.0)  # Left
    create_wall(x_center= 3.0, y_center= 0.0, half_ext_x=0.05, half_ext_y=3.0)  # Right

    # Create dividing wall with a slit in the middle
    create_wall(x_center=0, y_center= 1.7, half_ext_x=0.05, half_ext_y=1.3)  # Upper
    create_wall(x_center=0, y_center=-1.7, half_ext_x=0.05, half_ext_y=1.3)  # Lower

    # Create the cube block
    cube_id = create_cube_block(pos_x=-2, pos_y=0, size=0.2, mass=1.0)

    # Create two agents (small spheres)
    agent1_id = create_point_agent(pos_x=-2.0, pos_y= 2.0, radius=0.05, mass=0.1, color=[1,0,0,1])  # red
    agent2_id = create_point_agent(pos_x=-2.5, pos_y=-0.5, radius=0.05, mass=0.1, color=[0,0,1,1])  # blue

    p.setRealTimeSimulation(0)

    # Tracking whether each agent is currently grabbing the cube
    agent1_grabbing = False
    agent2_grabbing = False

    # Store constraint IDs for each agent if they grab the cube
    agent1_constraint_id = None
    agent2_constraint_id = None

    # Parameters
    grab_distance_threshold = 1.3  # max distance for an agent to be able to grab the cube
    agent_speed = 0.50             # speed for each agent (adjust as desired)

    try:
        while True:
            keys = p.getKeyboardEvents()

            # ------------------------------------------------------------
            # Agent 1 movement: Arrow keys
            # ------------------------------------------------------------
            agent1_vx, agent1_vy = 0.0, 0.0
            if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
                agent1_vy += agent_speed
               
            if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
                agent1_vy -= agent_speed
               
            if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
                agent1_vx -= agent_speed
               
            if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
                agent1_vx += agent_speed
               

            # ------------------------------------------------------------
            # Agent 2 movement: W, A, S, D
            # ------------------------------------------------------------
            agent2_vx, agent2_vy = 0.0, 0.0
            if keys.get(ord('q')) == p.KEY_IS_DOWN or keys.get(ord('Q')) == p.KEY_IS_DOWN:
                agent2_vy += agent_speed
            if keys.get(ord('z')) == p.KEY_IS_DOWN or keys.get(ord('Z')) == p.KEY_IS_DOWN:
                agent2_vy -= agent_speed
            if keys.get(ord('a')) == p.KEY_IS_DOWN or keys.get(ord('A')) == p.KEY_IS_DOWN:
                agent2_vx -= agent_speed
            if keys.get(ord('d')) == p.KEY_IS_DOWN or keys.get(ord('D')) == p.KEY_IS_DOWN:
                agent2_vx += agent_speed

            # ------------------------------------------------------------
            # Set the base velocity for each agent
            # ------------------------------------------------------------
            p.resetBaseVelocity(agent1_id, linearVelocity=[agent1_vx, agent1_vy, 0], angularVelocity=[0, 0, 0])
            p.resetBaseVelocity(agent2_id, linearVelocity=[agent2_vx, agent2_vy, 0], angularVelocity=[0, 0, 0])
            p.setPhysicsEngineParameter(numSolverIterations=200)   # default ~50
            p.setPhysicsEngineParameter(solverResidualThreshold=1e-9)
            p.setPhysicsEngineParameter(erp=0.2)   # error-reduction parameter (default 0.05)



            # ------------------------------------------------------------
            # Agent 1 Grab/Release: 'r' or 'R'
            # ------------------------------------------------------------
            if (keys.get(ord('c')) ==3 or keys.get(ord('C')) ==3):
            # if dist < grab_distance_threshold:
                if not agent1_grabbing:
                    # If not currently grabbing, check distance
                    dist = distance_between_bodies(agent1_id, cube_id)
                    if dist < grab_distance_threshold:
                        # Close enough to grab
                        agent1_grabbing = True
                        print(f"Distance between agent1 and cube:{dist}, and agent1 is grabbing")

                        # Optional: choose pivot in world coords (here, use mid-point between agent and cube)
                        agent_pos, _ = p.getBasePositionAndOrientation(agent1_id)
                        cube_pos, _  = p.getBasePositionAndOrientation(cube_id)

                        

                        agent1_constraint_id = create_fixed_constraint(
                            agent1_id, cube_id
                        )
                else:
                    # Currently grabbing => release
                    if agent1_constraint_id is not None:
                        p.removeConstraint(agent1_constraint_id)
                        agent1_constraint_id = None
                    agent1_grabbing = False
                    print(f" agent1 released BOX")


            # ------------------------------------------------------------
            # Agent 2 Grab/Release: 'h' or 'H'
            # ------------------------------------------------------------
            if (keys.get(ord('x')) == 3 or keys.get(ord('X')) ==3 ):
            # if dist < grab_distance_threshold:
                if not agent2_grabbing:
                    # If not currently grabbing, check distance
                    dist = distance_between_bodies(agent2_id, cube_id)
                    if dist < grab_distance_threshold:
                        agent2_grabbing = True
                        agent_pos, _ = p.getBasePositionAndOrientation(agent2_id)
                        cube_pos, _  = p.getBasePositionAndOrientation(cube_id)

                       
                        agent2_constraint_id = create_fixed_constraint(
                            agent2_id, cube_id
                        )
                else:
                    # Currently grabbing => release
                    if agent2_constraint_id is not None:
                        p.removeConstraint(agent2_constraint_id)
                        agent2_constraint_id = None
                    agent2_grabbing = False

            # Step simulation
            p.stepSimulation()
            time.sleep(1./240.)

    except KeyboardInterrupt:
        pass

    p.disconnect()

if __name__ == "__main__":
    main()
