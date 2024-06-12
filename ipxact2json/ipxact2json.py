import xml.etree.ElementTree as ET
import json
import re
import os
import argparse

def get_namespace(element):
    m = re.match(r'\{.*\}', element.tag)
    return m.group(0) if m else ''

def parse_ports(ports_element, ns):
    ports = []
    for port in ports_element.findall(f'{ns}port'):
        direction = port.find(f'{ns}wire/{ns}direction').text

        if direction == "in":
            direction = "input"
        elif direction == "out":
            direction = "output"            

        port_name = port.find(f'{ns}name').text

        vector = port.find(f'{ns}wire/{ns}vectors/{ns}vector')
        if vector is None:
            vector = port.find(f'{ns}wire/{ns}vector')

        if vector is not None:
            left = vector.find(f'{ns}left').text
            right = vector.find(f'{ns}right').text
            width = f"{left}:{right}"
        else:
            width = "1"
        
        ports.append({
            'name': port_name,
            'width': width,
            'direction': direction
        })
    return ports

def ipxact_to_json(ipxact_file, group_name):
    tree = ET.parse(ipxact_file)
    root = tree.getroot()
    ns = get_namespace(root)
    # 检测命名空间是否为ipxact或spirit
    ns_tag = 'ipxact' if 'ipxact' in ns else 'spirit'
    nsmap = {ns_tag: ns[1:-1]}  # 去除大括号
    # 根据检测到的命名空间前缀来查找元素
    description_element = root.find(f'{ns_tag}:description', nsmap)

    component = {
        'ipName': root.find(f'{ns_tag}:name', nsmap).text,
        'style': {
            'width': 80,
            'height': 60,
            'borderRadius': "10",
            'fontSize': 12,
            'fontColor': "#fff",
            'stroke': "#926390",
            'fill': "#926390"
        },
        'content': {
            'category': "Peripheral",
            'IP_full_name': root.find(f'{ns_tag}:name', nsmap).text,
            'vendor': root.find(f'{ns_tag}:vendor', nsmap).text,
            'version': root.find(f'{ns_tag}:version', nsmap).text,
            #'description': root.find(f'{ns_tag}:model/{ns_tag}:instantiations/{ns_tag}:componentInstantiation/{ns_tag}:description', nsmap).text,
            'description': description_element.text if description_element is not None else None,
            'parameters': {},
            'defines': "",
            "ports": {}
        }
    }

    # Parse parameters
    for param in root.findall(f'{ns_tag}:model/{ns_tag}:instantiations/{ns_tag}:componentInstantiation/{ns_tag}:moduleParameters/{ns_tag}:moduleParameter', nsmap):
        name = param.find(f'{ns_tag}:name', nsmap).text
        value = param.find(f'{ns_tag}:value', nsmap).text
        component['content']['parameters'][name] = value
        

    # xilinx ip parameters parser
    for param in root.findall(f'{ns_tag}:parameters/{ns_tag}:parameter', nsmap):
        name = param.find(f'{ns_tag}:name', nsmap).text
        value = param.find(f'{ns_tag}:value', nsmap).text
        component['content']['parameters'][name] = value

    # Parse ports
    ports = parse_ports(root.find(f'{ns_tag}:model/{ns_tag}:ports', nsmap), ns)
    for port in ports:
        component['content']['ports'][port['name']] = {
            'width':port['width'],
            'direction':port['direction']
        }

    return component

def process_files(files, group_name, output_file):
    group_data = []  # 初始化groupData数组
    for file in files:
        component = ipxact_to_json(file, group_name)
        group_data.append(component)  # 将每个解析出的IP信息添加到groupData数组中

    ip_lib = {"result": [{"groupName": group_name, "groupData": group_data}]}  # 使用单个group包裹所有IP信息

    with open(output_file, 'w') as json_file:
        json.dump(ip_lib, json_file, indent=4)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert IP-XACT XML files to JSON.')
    parser.add_argument('-f', '--file', help='Single IP-XACT XML file to process')
    parser.add_argument('-l', '--list', help='File containing a list of IP-XACT XML files to process')
    parser.add_argument('-o', '--output', required=True, help='Output JSON file name')
    parser.add_argument('-g', '--group', default='IP_lib', help='Group name for the IPs in the JSON file')
    args = parser.parse_args()

    files_to_process = []
    if args.file:
        files_to_process.append(args.file)
    elif args.list:
        with open(args.list, 'r') as filelist:
            files_to_process.extend([line.strip() for line in filelist.readlines() if line.strip().endswith('.xml') and not line.strip().startswith('#')])

    if not files_to_process:
        parser.error('No input files provided. Use -f or -l option.')

    process_files(files_to_process, args.group, args.output)
