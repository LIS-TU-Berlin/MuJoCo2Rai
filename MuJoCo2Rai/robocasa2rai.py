from muj2rai import *



CONVERT_AIGEN = False
CONVERT_FIXTURES = True

def main():

    
    # This program demonstrates the conversion of jointed RoboCasa objects, (in /fixtures) and /aigen_objs)


    # converting .obj files with textures to corresponding .h5 and .plys with corrected CoM and Inertia, 
    if CONVERT_AIGEN:           # TAKES VERY LONG FOR AIGEN_OBJS DIR
        convert_obj_to_ply(root_path="../aigen_objs", output_base="../rai_plys")
    elif CONVERT_FIXTURES:
        process_model_xml(root_path="../fixtures", output_base="../rai_jointed")

if __name__ == "__main__":
    main()