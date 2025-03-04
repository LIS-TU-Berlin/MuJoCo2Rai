from load_mujoco import *
import yaml
import json
import sysconfig

def test_fixtures():
    root_path = "../fixtures"
    for root, dirs, files in os.walk(root_path):
        for file in files:
            if file == "model.xml":
                print('=====================', file)
                C = add_mujoco(root, file)
                print('loaded frames: ', C.getFrameNames())
                C.frame(0) .setPosition([0,0,1]) .setQuaternion([0,0,0,1])
                C.view(True)

def test_kitchen_franka():
    pysite = sysconfig.get_paths()["purelib"]
    file = pysite+"/gymnasium_robotics/envs/assets/kitchen_franka/kitchen_assets/kitchen_env_model.xml"
    file = '../kitchen_dataset/RUSTIC_ONE_WALL_SMALL.xml'
    print('=====================', file)
    # M = Mujoco2Dict(file)
    # for key, value in M.D.items():
    #     print(f'{key}: {value}')
    #print(yaml.dump(M.D))#, default_flow_style=True))
    # return
    M = MujocoLoader(file, visualsOnly=True, processMeshes=False)
    M.base.setQuaternion([0,0,0,1])
    M.base.setPosition([3,0,0])
    print('loaded #frames: ', M.C.getFrameDimension())
    os.system('rm -Rf meshes/')
    M.C.writeMeshes('meshes/')
    M.C.simplify(True, False, True)
    print('simplified #frames: ', M.C.getFrameDimension())
    with open("z.g", "w") as fil:
        #yaml.dump(M.C.asDict(), file, default_flow_style=False)
        fil.write(M.C.write())
    for i in range(1):
        M.C.view(True)
        M.C.animate()

def main():
    test_kitchen_franka()

if __name__ == "__main__":
    main()