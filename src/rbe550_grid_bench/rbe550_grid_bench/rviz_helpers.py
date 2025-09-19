from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def path_marker(path_w, frame_id="map", mid=0):
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "planned_path"
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.03
    m.color = ColorRGBA(0.0, 1.0, 0.0, 0.9)
    for x,y in path_w:
        p = Point(x=x, y=y, z=0.0)
        m.points.append(p)
    return m

def expansions_marker(exp_points, frame_id="map", mid=1):
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "expansions"
    m.id = mid
    m.type = Marker.POINTS
    m.action = Marker.ADD
    m.scale.x = 0.03
    m.scale.y = 0.03
    m.color = ColorRGBA(1.0, 0.0, 0.0, 0.6)
    for x,y in exp_points:
        m.points.append(Point(x=x, y=y, z=0.0))
    return m

