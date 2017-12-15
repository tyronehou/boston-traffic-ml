import geojson
from geoql import geoql
import geoleaflet
import xml.etree.cElementTree as ET
import numpy as np

data_dir = '../datasets/geographical/'

# Need to update
def create_xml_node(node_id, x, y, node_type="priority"):

    node = ET.Element("node",
                    id=node_id,
                    x=x,
                    y=y,
                    type=node_type
    )

    return node

# Need to update
def create_xml_edge(edge_id, from_node, to_node, numLanes="2"):
    ''' See sumo edge definition for more definition of road types:
        http://www.sumo.dlr.de/userdoc/Networks/Building_Networks_from_own_XML-descriptions.html#Type_Descriptions
        Don't forget about priority
    '''
    edge = ET.Element("edge",
                    id=edge_id,
                    to=to_node,
                    numLanes=numLanes
    )
    # Since from is a weirdly parsed keyword, it has to be set separately

    edge.set("from", from_node)
    ET.dump(edge)
    return edge

def to_sumo_net(V, E):
    ''' Given a geojson graph representation (from geoql) converts to a sumo simulation
        Sumo has a special xml representation that should not be generated manually.
        Instead, generate a manual xml spec that sumo will convert
        See http://www.sumo.dlr.de/userdoc/Networks/Building_Networks_from_own_XML-descriptions.html for more details
    '''
    # Create node xml
    nodes_root = ET.Element("nodes")
    node_ids = {}
    k = 0
    for j, v in enumerate(V):
        i = str(k)
        node_ids[tuple(v['coordinates'])] = i

        x, y = v['coordinates']
        node = create_xml_node(i, str(x), str(y))
        nodes_root.append(node)
        k += 1

    # Create edge xml
    edges_root = ET.Element("edges")
    for e in E:
        coords = e['geometry']['coordinates']
        for i in range(1, len(coords)):
            v1, v2 = coords[i-1], coords[i]
            if v1[0] == v2[0] and v1[1] == v2[1]:
                print('Found zero length edge')
                continue

            if (tuple(v1) not in node_ids) and (tuple(v2) not in node_ids):
                print('Edge contains two suspicious nodes {} and {}'.format(v1, v2))
                continue

            #print('Node {} missing from node set'.format(e))

            try:
                from_node = node_ids[tuple(v1)]
            except KeyError as e:
                i = str(k)
                node_ids[tuple(v1)] = i
                node = create_xml_node(i, str(v1[0]), str(v1[1]))
                nodes_root.append(node)
                from_node = i
                k += 1

            try:
                to_node = node_ids[tuple(v2)]
            except KeyError as e:
                i = str(k)
                node_ids[tuple(v2)] = i
                node = create_xml_node(i, str(v2[0]), str(v2[1]))
                nodes_root.append(node)
                to_node = i
                k += 1

            edge_id = '{}to{}'.format(from_node, to_node)
            edge_id_ = '{}to{}'.format(to_node, from_node)

            # Create two way edge by default for now
            edge = create_xml_edge(edge_id, from_node, to_node)
            edge_ = create_xml_edge(edge_id_, to_node, from_node)
            edges_root.append(edge)
            edges_root.append(edge_)

    nodes = ET.ElementTree(nodes_root)
    edges = ET.ElementTree(edges_root)

    return nodes, edges

def main():
    # We take a small area of Boston bounded by the p1, p2, p3, & p4 (corresponding roughly to the area bounded by fenway, the Charles River, and Boston Commons)
    # geoql requires longitude then latitude
    p1 = [-71.090084, 42.351659]
    p2 = [-71.073152, 42.356235]
    p3 = [-71.070418, 42.351718]
    p4 = [-71.087829, 42.347133]

    bounds = {
        'features': [
            {
                'type':'Feature',
                'geometry': {
                    'type':'Polygon',
                    'coordinates': [[p1, p2, p3, p4, p1]]
                }
            }
        ],
        'type': 'FeatureCollection'
    }
    '''
    g = geoql.load(open(data_dir + 'example_extract.geojson', 'r'))
    g = g.keep_that_intersect(bounds)
    g.dump(open(data_dir + 'sumo_traffic_net.geojson', 'w'))
    open('leaflet_sumo.html', 'w').write(geoleaflet.html(g)) # Create visualization.
    '''
    g = geoql.load(open(data_dir + 'sumo_traffic_net.geojson', 'r'))
    g = g.node_edge_graph()

    # Separate vertex and edge list
    # Also enlarge the distance between coordinates
    V, E = [], []

    distances = []
    for feature in g['features']:
        if feature['type'] == 'Feature':
            coords = feature['geometry']['coordinates']
            for i in range(1, len(coords)):
                x1, y1 = coords[i]
                x2, y2 = coords[i-1]
                dist = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
                if dist > 0:
                    distances.append(dist)


    scale = 100 / min(distances)
    print(scale)

    for feature in g['features']:
        if feature['type'] == 'Feature':
            coords = feature['geometry']['coordinates']
            for i, coord in enumerate(coords):

                coords[i] = (coord[0] * scale, coord[1] * scale)
                
            E.append(feature)
        elif feature['type'] == 'Point':
            coords = feature['coordinates']
            feature['coordinates'] = (coords[0] * scale, coords[1] * scale)
            V.append(feature)
        else:
            print('Invalid feature type {} in graph'.format(feature['type']))

    # Convert to a sumo representation
    nodes, edges = to_sumo_net(V, E)

    nodes.write('../sumo_files/traffic_ml.nod.xml')
    edges.write('../sumo_files/traffic_ml.edg.xml')

if __name__ == '__main__':
    main()