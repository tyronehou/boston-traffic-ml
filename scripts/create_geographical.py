import geojson
from geoql import geoql
import geoleaflet

data_dir = '../datasets/geographical/'
g = geoql.load(open(data_dir + 'Boston_Segments.geojson'))

g = g.properties_null_remove()\
     .tags_parse_str_to_dict()\
#     .keep_by_property({"highway": {"$in": ["residential", "secondary", "tertiary"]}})
g = g.node_edge_graph() # Converted into a graph with nodes and edges.
g.dump(open(data_dir + 'example_extract.geojson', 'w'))
open('leaflet.html', 'w').write(geoleaflet.html(g)) # Create visualization.