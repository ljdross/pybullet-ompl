# This file was created with the help of ChatGPT

import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom


def merge_urdf_files(file1: str, file2: str, output_file: str):
    # Parse the first XML file
    tree1 = ET.parse(file1)
    root1 = tree1.getroot()

    # Parse the second XML file
    tree2 = ET.parse(file2)
    root2 = tree2.getroot()

    # Create a new root element for the merged XML
    merged_root = ET.Element("robot")
    merged_root.text = "\n"  # Add a newline after the opening tag

    # Add attributes from the second XML file root element to the merged root element
    for key, value in root2.attrib.items():
        merged_root.set(key, value)

    # Add elements from the first XML file to the merged XML
    for element in root1:
        merged_root.append(element)

    # Add elements from the second XML file to the merged XML
    for element in root2:
        element_name = element.attrib.get("name")
        if element_name != "base_link":
            merged_root.append(element)

    # Create a new XML tree with the merged root element
    merged_tree = ET.ElementTree(merged_root)

    # Write the merged XML tree to a string
    merged_xml_str = ET.tostring(merged_root, encoding="utf-8").decode("utf-8")

    # Reformat the XML string with proper indentation
    merged_dom = minidom.parseString(merged_xml_str)
    merged_xml = merged_dom.toprettyxml(indent="  ")

    # Write the merged XML to the output file
    with open(output_file, "w", encoding="utf-8") as file:
        file.write(merged_xml)
