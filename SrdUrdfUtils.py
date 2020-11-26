from urdf_parser_py.urdf import URDF
#"C:\\Users\\ржомба\\Desktop\\work\\SDR-examples\\URDF import examples\\iiwa\\iiwa14.urdf"
def getLinkArrayFromUrdf(path):
    robot = URDF.from_xml_file(path)
    for link in robot.links:
        print(link)

