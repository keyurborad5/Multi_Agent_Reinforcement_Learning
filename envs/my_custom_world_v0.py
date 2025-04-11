import pybullet as p
import pybullet_data
import time
import math

def create_wall(x_center, y_center, half_ext_x, half_ext_y, height=0.4):
    """
    Helper function to create a wall (box).
    x_center, y_center: center position in XY plane
    half_ext_x, half_ext_y: half extents in x,y for the wall
    height: height of the wall
    """
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_ext_x, half_ext_y, height/2.0]
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_ext_x, half_ext_y, height/2.0],
        rgbaColor=[0.8, 0.8, 0.8, 1]
    )
    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x_center, y_center, height/2.0]
    )
    return body_id

def create_cube_block(pos_x, pos_y, size=0.2, mass=1.0):
    """
    Creates a cube block in the XY plane (z=0).
    """
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size/2, size/2, size/2]
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size/2, size/2, size/2],
        rgbaColor=[0, 1, 0, 1]
    )
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos_x, pos_y, size/2]
    )
    return body_id

def create_point_agent(pos_x, pos_y, radius=0.05, mass=0.1):
    """
    Creates a small sphere to represent a "point" agent.
    Alternatively, you could use a very small cylinder or box.
    """
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=[1, 0, 0, 1]  # red color for differentiation
    )
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos_x, pos_y, radius]
    )
    return body_id

def main():
    # Connect to PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)

    # Set path to PyBullet’s built-in data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Create a plane (floor)
    plane_id = p.loadURDF("plane.urdf")

    # Set gravity
    p.setGravity(0, 0, -9.8)

    # --------------------------------------------------------------------------
    # 1. Create outer boundary walls
    #    Suppose the environment is a 6x6 square in the XY plane, from -3 to +3
    #    We'll create 4 walls around the edges.
    # --------------------------------------------------------------------------
    
    # Top boundary wall
    create_wall(x_center=0,    y_center= 3.0, half_ext_x=3.0, half_ext_y=0.05)
    # Bottom boundary wall
    create_wall(x_center=0,    y_center=-3.0, half_ext_x=3.0, half_ext_y=0.05)
    # Left boundary wall
    create_wall(x_center=-3.0, y_center= 0,   half_ext_x=0.05, half_ext_y=3.0)
    # Right boundary wall
    create_wall(x_center= 3.0, y_center= 0,   half_ext_x=0.05, half_ext_y=3.0)

    # --------------------------------------------------------------------------
    # 2. Create dividing wall with a slit
    #    Example: dividing wall is located at x=0, from y=-3 to y=3,
    #    but we leave a slit from y=-0.4 to y=0.4 for the cube to pass.
    #    
    #    One way to do this: create two box segments:
    #       * upper part of the wall  (y>0.4)
    #       * lower part of the wall  (y<-0.4)
    # --------------------------------------------------------------------------
    # Upper part
    create_wall(x_center=0, y_center= 1.7, half_ext_x=0.05, half_ext_y=1.3)
    # Lower part
    create_wall(x_center=0, y_center=-1.7, half_ext_x=0.05, half_ext_y=1.3)

    # --------------------------------------------------------------------------
    # 3. Create the cube block to be manipulated
    #    Start it on, say, the left side of the wall
    # --------------------------------------------------------------------------
    cube_id = create_cube_block(pos_x=-2, pos_y=0, size=0.2, mass=1.0)

    # --------------------------------------------------------------------------
    # 4. Create two point-like agents
    #    For example, place them on the same side near the cube
    # --------------------------------------------------------------------------
    agent1_id = create_point_agent(pos_x=-2.5, pos_y= 0.5, radius=0.05, mass=0.1)
    agent2_id = create_point_agent(pos_x=-2.5, pos_y=-0.5, radius=0.05, mass=0.1)

    # (Optional) Turn off real-time to better control stepping if needed
    p.setRealTimeSimulation(0)

    # Simple simulation loop
    try:
        while True:
            p.stepSimulation()
            # keys = p.getKeyboardEvents()
            # print(f"Key Triggered : {p.KEY_WAS_TRIGGERED},Key is Down {p.KEY_IS_DOWN},Key Released: {p.KEY_WAS_RELEASED}")
            # for k,v in keys.items():
            #     print(k,v)
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass

    p.disconnect()

if __name__ == "__main__":
    main()
