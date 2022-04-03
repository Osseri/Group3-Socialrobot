from sensor_msgs.msg import PointCloud2, PointField
import rospy
import numpy as np 
from scipy.spatial import KDTree

topic = 'visualization_marker_array'
pt_pub = rospy.Publisher("arm_workspace_pts", PointCloud2, queue_size=2)


def create_workspace_surface_points(n, r, origin):
    print("Generating fixed %d points on a sphere centered at the origin" %n)
    alpha = 4.0*np.pi*r*r/n
    d = np.sqrt(alpha)
    m_nu = int(np.round(np.pi/d))
    d_nu = np.pi/m_nu
    d_phi = alpha/d_nu
    count = 0

    points = []
    for m in range (0,m_nu):
        nu = np.pi*(m+0.5)/m_nu
        m_phi = int(np.round(2*np.pi*np.sin(nu)/d_phi))
        for n in range (0,m_phi):
            phi = 2*np.pi*n/m_phi
            x = r*np.sin(nu)*np.cos(phi)
            y = r*np.sin(nu)*np.sin(phi)
            z = r*np.cos(nu)            
            count = count +1
            
            points.append([x+origin[0],y+origin[1],z+origin[2]])

    return np.array(points)

def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None):
    '''
    Create a sensor_msgs.PointCloud2 from an np array of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

def find_closest_points(point_array, point, num_pt=3):
    tree = KDTree(point_array, leafsize=10)
    dist, ind = tree.query(point, k=num_pt)

    closest_points = []
    for i in ind:
        closest_points.append(list(point_array[i]))
    return closest_points

if __name__ == '__main__':

    rospy.init_node('test_sphere')
    right_shoulder_frame = [0.128319, -0.175567, 0.887906]
    left_shoulder_frame = [0.128319, 0.164856, 0.895184]
    length = 0.45
    points = create_workspace_surface_points(500, length, right_shoulder_frame)
    closest_points = find_closest_points(points, [0,-1,1], num_pt=10)
    print(closest_points)
    
    pc2 = xyz_array_to_pointcloud2(np.array(closest_points), stamp=rospy.Time.now(), frame_id='base_footprint')

    while not rospy.is_shutdown():

        # Publish the MarkerArray
        pt_pub.publish(pc2)
        rospy.sleep(0.01)