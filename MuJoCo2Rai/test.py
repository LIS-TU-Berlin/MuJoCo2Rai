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
    print('=====================', file)
    M = MujocoLoader(file)
    # print('loaded frames: ', M.C.getFrameNames())
    for i in range(2):
        M.C.view(True)
        M.C.animate()

def main():
    test_kitchen_franka()

if __name__ == "__main__":
    main()