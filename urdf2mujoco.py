import mujoco

def main():
    modelName = input('Input a valid input filename with .urdf extension')
    model = mujoco.MjModel.from_xml_path(modelName)

    saveName = input('Input a valid output filename with .xml extension')
    mujoco.mj_saveLastXML(saveName, model)

    print(f'Converted from {modelName} to {saveName}')

if __name__=='__main__':
    main()