from load_mujoco import *
import yaml
import json
import sysconfig

def test_texture():
    C = ry.Config()
    f = C.addFrame('box'). setShape(ry.ST.box, [.2, .2, .2]) .setPosition([0, 0, 1])
    f.setTextureFile('../../rai/test/Gui/retired/opengl/box.png',  np.random.rand(8,2))
    C.view(True)

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
    # file = pysite+"/gymnasium_robotics/envs/assets/kitchen_franka/kitchen_assets/kitchen_env_model.xml"
    # file = '../kitchen_dataset/RUSTIC_ONE_WALL_SMALL.xml'
    file = '../kitchen_dataset/FARMHOUSE_U_SHAPED_LARGE.xml'

    print('=====================', file)
    M = MujocoLoader(file, visualsOnly=True, processMeshes=True)
    M.base.setQuaternion([0,0,0,1])
    M.base.setPosition([1,0,0])
    ry.params_print()
    print('loaded #frames: ', M.C.getFrameDimension())
    # os.system('rm -Rf meshes/')
    # M.C.writeMeshes('meshes/')
    M.C.simplify(True, False, True)
    print('simplified #frames: ', M.C.getFrameDimension())
    with open("z.g", "w") as fil:
        #yaml.dump(M.C.asDict(), file, default_flow_style=False)
        fil.write(M.C.write())
    for i in range(1):
        M.C.view(True)
        M.C.animate()

def main():
    # test_texture()
    test_kitchen_franka()

if __name__ == "__main__":
    main()