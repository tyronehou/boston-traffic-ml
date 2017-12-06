import json
import re
import calendar
from datetime import datetime
import os
import glob
import logging
from orderedset import OrderedSet

NEWS = {'N': 'North', 'E': 'East', 'W': 'West', 'S': 'South', 'B': ''}
lane_names = {'L':'Left', 'T':'Thru', 'R':'Right', 'RR':'Hard Right', 'LL':'Hard Left', 'TT':'Hard Thru'}
months = {month_name.lower():str(i) for i,month_name in enumerate(calendar.month_abbr)}

traffic_data_dir = os.path.join(os.pardir, 'datasets', 'traffic_count_data')

def month_to_number(month):
    month = month.strip()[:3].lower()
    return months[month]

def splitnorm(s, *splitparams):
    ''' Splits string s by sep and removes all empty strings in the list '''
    return [elem for elem in s.split(*splitparams) if elem]

def convertjson(csvname):
    d = {'features': {},
         'properties': {
            'Count Times': []
         }
         }

    with open(csvname, 'r') as f:
        counts_csv = f.read().split('\n')

    if "Traffic Movement Summary Table" in counts_csv[1]:
        d = convertjson2(counts_csv, d)
    else:
        d = convertjson1(counts_csv, d)

    return d

def convertjson1(counts_csv, d):
    ''' Converts specific csv file to json file '''
    # METAINFO
    for i, line in enumerate(counts_csv[8:]):
        if "Start Time" in line:
            start_time_idx = 8 + i
            break

    metainfo = counts_csv[:8]
    street_headers = ' '.join(counts_csv[8:start_time_idx])
    lane_headers = counts_csv[start_time_idx]

    for line in metainfo:
        # find properties e.g. Comment 1: something
        attr = re.search(r'"([^:]*):",*"(.*)"', line)
        if attr:
            k, v = attr.groups()
        else:
            k, v = splitnorm(line, ',')
            k = k[1:-2] # remove quotes and colon from keyh

        d['properties'][k] = v

    
    # STREET NAMES
    # Find the street names and directions
    street_names = re.findall(r'"(.*?) From (.*?)",*', street_headers)
    
    # Initialize empty feature dictionaries
    for name, direction in street_names:
        d['features'].setdefault(name, {}) \
                    .setdefault(direction, {})

    lane_names = splitnorm(lane_headers, ',')[1:]
    #lane_names = lane_headers.split(',')[1:]
    #lane_names = splitnorm(lane_names)

    cycle = len(lane_names) / len(street_names)

    # TRAFFIC COUNTS
    for line in counts_csv[start_time_idx+1:]:
        if not line: continue
        #if 'PM' not in line: continue # To handle special case when more counts are included
        counts = splitnorm(line, ',')[:len(lane_names)]
        if not counts:
            break


        # Append list of all times counts were recorded without the quotes
        # Reformat time as 24 hour count
        count_time = datetime.strptime(counts[0][1:-1], "%I:%M %p").strftime("%H:%M")
        d['properties']['Count Times'].append(count_time)
        
        # t is Right, Thru, Left, or U-Turn
        for t, col in enumerate(counts[1:]):
            street, direction = street_names[int(t // cycle)]
            turn = lane_names[t]

            d['features'][street][direction].setdefault(turn, []).append(col)

    return d #json.dumps(d)

def convertjson2(counts_csv, d):
    ''' Converts specific csv file to json file '''
    # Example csv #35.0
    # Ignore the first two lines
    
    metainfo2 = re.search(r'Location:,+"?(.*)"?,+".*#\D*(\d*)"', counts_csv[2])
    location, btd_int = metainfo2.groups()

    metainfo3 = re.search(r'(\w*),+"?([\w \s]*)"?\W*(Date)?.*?(([a-zA-Z]+)\D*(\d{1,2})\D*(\d{2,4})|(\d{1,2})[., ](\d{1,2})[., ](\d{2,4}))', counts_csv[3])
    vehicle_type, region, _, _, m1, d1, y1, m2, d2, y2 = metainfo3.groups() # ignore Date header and full date

    if m1:
        if len(y1) == 2: y1 = '20' + y1
        date = '{:0>2}/{:0>2}/{}'.format(month_to_number(m1), d1, y1)
    else:
        # Gotta zero pad d2
        if len(y2) == 2: y2 = '20' + y2
        date = '{:0>2}/{:0>2}/{}'.format(m2, d2, y2)

    d['properties']['Location'] = location
    d['properties']['BTD Int'] = btd_int
    d['properties']['Vehicle Type'] = vehicle_type
    d['properties']['Region'] = region
    d['properties']['Start Date'] = date
    
    street_names = re.findall(r'"(.*?)"', counts_csv[6])
    lanes = counts_csv[8].split(',') # There may be empty lanes
    # map lane numbers to street names
    street_numbers = OrderedSet(re.findall(r'(\d)', counts_csv[8]))

    directions = [NEWS[d1]+NEWS[d2].lower() for d1, d2 in re.findall(r'([NEWS]).*?([EWB]).*?on', counts_csv[5])] # keep format same for all files
    directions = [directions[int(i)-1] for i in street_numbers]
    street_order = dict(zip(street_numbers, zip(street_names, directions)))

    for name, direction in zip(street_names, directions):
        d['features'].setdefault(name, {}) \
                    .setdefault(direction, {})

    for line in counts_csv[9:]:
        # Skip data lines not beginning with times like 17:00:00
        if not re.match(r'\d\d:\d\d:\d\d', line):
            continue

        counts = line.split(',')

        # Append list of all times counts without seconds info
        count_time = counts[0][:-3]
        d['properties']['Count Times'].append(count_time)
        
        # LET it be the shorer of the two
        for t, col in enumerate(counts[1:]):
            # if either lane title or count is empty, ignore
            if col == '' or lanes[t] == '':
                continue

            # Ignore everything that's not left, right, or thru
            if not re.match(r'\d[lrtLRT]*', lanes[t]):
                continue

            street_num = lanes[t][0]
            turn_abbr = lanes[t][1:]
            street, direction = street_order[street_num]
            turn = lane_names[turn_abbr]
            d['features'][street][direction].setdefault(turn, []).append(col)
    
    return d

debug = True
if not debug:
    logging.basicConfig(filename='exceptions.log',level=logging.DEBUG)
    for csv_path in glob.glob(os.path.join(traffic_data_dir, 'csv_data', '*.csv.*')):
        fname, fnum = os.path.basename(csv_path).split('.csv.')
        fname = os.path.join(traffic_data_dir, 'json_data', '{}.{}.{}'.format(fname, 'json', fnum))

        if int(fnum) > 3:
            continue
        if os.path.exists(fname):
            print(fname, 'already exists')
            continue

        try:
            d=convertjson(csv_path)
        except Exception as e:
            print('Exception:', e)
            logging.debug(os.path.basename(csv_path) + 'Exception: ' + str(e))
            continue

        with open(fname, 'w') as f:
            f.write(json.dumps(d, sort_keys=True))
else:
    #json1
    #f =  '../datasets/traffic_count_data/csv_data/235;DORCHESTER_AVE,_TALBOT_AVE.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/1070;ESSEX_ST,_HARRISON_AVE.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/1093;MASSACHUSETTS_AVE,_MELNEA_CASS_BLVD,_SOUTHAMPTON_ST.csv.2'

    #json2
    #f = '../datasets/traffic_count_data/csv_data/1746;DORCHESTER_AVE,_LINDEN_ST.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/204;ALBANY_ST,_NORTHAMPTON_ST.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/237;RIVER_ST,_WEST_ST.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/109;CAUSEWAY_ST,_LOMASNEY_WAY,_MERRIMAC_ST,_STANIFORD_ST.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/883;HYDE_PARK_AVE,_NEPONSET_AVE.csv.0'
    f = '../datasets/traffic_count_data/csv_data/1143;BOYLSTON_ST,_LAMARTINE_ST.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/1728;BISMARCK_ST,_BROCKTON_ST,_CUMMINS_HWY.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/1128;D_ST,_DORCHESTER_AVE.csv.0'
    #f = '../datasets/traffic_count_data/csv_data/1017;CEDAR_ST,_CENTRE_ST,_FORT_AVE.csv.0'
    d = convertjson(f)
    print(d)    

