import subprocess
import xml.etree.ElementTree as ET

def get_xacro_properties(xacro_path):
    """
    Parse a Xacro file and return a dictionary of all <xacro:property> values.
    Only supports scalar (float) values.
    """
    try:
        result = subprocess.run(
            ['xacro', xacro_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True
        )
        xml_str = result.stdout.decode('utf-8')
        root = ET.fromstring(xml_str)

        props = {}
        for prop in root.findall(".//property"):
            name = prop.attrib.get("name")
            value = prop.attrib.get("value")
            if name and value:
                props[name] = float(value)
        return props

    except Exception as e:
        raise RuntimeError(f"[get_xacro_properties] Failed to parse {xacro_path}: {e}")
