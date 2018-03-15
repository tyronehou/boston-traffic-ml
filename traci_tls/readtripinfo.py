import xml.etree.ElementTree as ET
import argparse

def get_arguments():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("fname")

    args = argParser.parse_args()
    return args

def parse_xml(fname):
    with open(fname) as f:
        output_xml = f.read()
    
        # Sumo seems to write incomplete xml files, so we lose the cars at the very end, but the general data trend shouldn't be affected
         
        output_xml = output_xml[:-1].rsplit("\n", 1)
        output_xml = output_xml[0]
        if "</tripinfos>" not in output_xml[1]:
            output_xml += "\n</tripinfos>"
    
        #print('Approximate vehicle count:', len(output_xml.split("\n")))
    
        root = ET.fromstring(output_xml)
    return root

def get_cum_attrib(root, attrib):
    cum_time_loss = 0
    for vehicle in root.getchildren():
        cum_time_loss += float(vehicle.attrib[attrib])
    
    return cum_time_loss

def get_avg_attrib(root, attrib):
    avg_time_loss = 0
    for vehicle in root.getchildren():
        avg_time_loss += float(vehicle.attrib[attrib])
    return avg_time_loss / len(root.getchildren())
    
def countTypes(root, attrib):
    d = {}
    for vehicle in root.getchildren():
        t = vehicle.attrib[attrib]
        d.setdefault(t, 0)
        d[t] += 1
    return d

def cumTravelTime(root):
    travel_time = 0.
    for vehicle in root.getchildren():
        travel_time +=  float(vehicle.attrib["arrival"]) - float(vehicle.attrib["depart"])
    return travel_time

def avgTravelTime(root):
    travel_time = 0.
    for vehicle in root.getchildren():
        travel_time +=  float(vehicle.attrib["arrival"]) - float(vehicle.attrib["depart"])
    return travel_time / len(root.getchildren())

if __name__ == '__main__':
    args = get_arguments()
    root = parse_xml(args.fname)
    print(avgTravelTime(root))
    print(cumTravelTime(root))
 
    #print(get_loss(root))
    #print(get_avg_delay(root))
