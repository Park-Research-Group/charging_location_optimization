import csv
links = set()
with open('dtalite_inputs/input_link.csv', newline='') as fh:
    for r in csv.DictReader(fh):
        links.add((r['from_node_id'], r['to_node_id']))
total = missing = 0
with open('dtalite_inputs/link_attributes.csv', newline='') as fh:
    for r in csv.DictReader(fh):
        if r['oneway'] == '0':
            total += 1
            if (r['to_node_id'], r['from_node_id']) not in links:
                missing += 1
print('two-way rows:', total, '| missing reverse:', missing)
