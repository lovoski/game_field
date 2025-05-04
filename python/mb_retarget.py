# -*- coding: utf-8 -*-
import os
import json
from pyfbsdk import *

mobuMap = {
    # "Reference": "reference",
    "Hips": "Hips",
    "LeftUpLeg": "LeftUpLeg",
    "LeftLeg": "LeftLeg",
    "LeftFoot": "LeftFoot",
    "RightUpLeg": "RightUpLeg",
    "RightLeg": "RightLeg",
    "RightFoot": "RightFoot",
    "Spine": "Spine",
    "LeftArm": "LeftArm",
    "LeftForeArm": "LeftForeArm",
    "LeftHand": "LeftHand",
    "RightArm": "RightArm",
    "RightForeArm": "RightForeArm",
    "RightHand": "RightHand",
    "Head": "Head",
    "LeftShoulder": "LeftShoulder",
    "RightShoulder": "RightShoulder",
    "Neck": "Neck",
    "Spine1": "Spine1",
    "Spine2": "Spine2",
    "Spine3": "Spine3",
    "Spine4": "Spine4",
    "Spine5": "Spine5",
    "Spine6": "Spine6",
    "Spine7": "Spine7",
    "Spine8": "Spine8",
    "Spine9": "Spine9",
    "Neck1": "Neck1",
    "Neck2": "Neck2",
    "Neck3": "Neck3",
    "Neck4": "Neck4",
    "Neck5": "Neck5",
    "Neck6": "Neck6",
    "Neck7": "Neck7",
    "Neck8": "Neck8",
    "Neck9": "Neck9",
    "LeftHandThumb1": "LeftHandThumb1",
    "LeftHandThumb2": "LeftHandThumb2",
    "LeftHandThumb3": "LeftHandThumb3",
    "LeftHandIndex1": "LeftHandIndex1",
    "LeftHandIndex2": "LeftHandIndex2",
    "LeftHandIndex3": "LeftHandIndex3",
    "LeftHandMiddle1": "LeftHandMiddle1",
    "LeftHandMiddle2": "LeftHandMiddle2",
    "LeftHandMiddle3": "LeftHandMiddle3",
    "LeftHandRing1": "LeftHandRing1",
    "LeftHandRing2": "LeftHandRing2",
    "LeftHandRing3": "LeftHandRing3",
    "LeftHandPinky1": "LeftHandPinky1",
    "LeftHandPinky2": "LeftHandPinky2",
    "LeftHandPinky3": "LeftHandPinky3",
    "RightHandThumb1": "RightHandThumb1",
    "RightHandThumb2": "RightHandThumb2",
    "RightHandThumb3": "RightHandThumb3",
    "RightHandIndex1": "RightHandIndex1",
    "RightHandIndex2": "RightHandIndex2",
    "RightHandIndex3": "RightHandIndex3",
    "RightHandMiddle1": "RightHandMiddle1",
    "RightHandMiddle2": "RightHandMiddle2",
    "RightHandMiddle3": "RightHandMiddle3",
    "RightHandRing1": "RightHandRing1",
    "RightHandRing2": "RightHandRing2",
    "RightHandRing3": "RightHandRing3",
    "RightHandPinky1": "RightHandPinky1",
    "RightHandPinky2": "RightHandPinky2",
    "RightHandPinky3": "RightHandPinky3",
    "LeftFootThumb1": "LeftFootThumb1",
    "LeftFootThumb2": "LeftFootThumb2",
    "LeftFootThumb3": "LeftFootThumb3",
    "LeftFootIndex1": "LeftFootIndex1",
    "LeftFootIndex2": "LeftFootIndex2",
    "LeftFootIndex3": "LeftFootIndex3",
    "LeftFootMiddle1": "LeftFootMiddle1",
    "LeftFootMiddle2": "LeftFootMiddle2",
    "LeftFootMiddle3": "LeftFootMiddle3",
    "LeftFootRing1": "LeftFootRing1",
    "LeftFootRing2": "LeftFootRing2",
    "LeftFootRing3": "LeftFootRing3",
    "LeftFootPinky1": "LeftFootPinky1",
    "LeftFootPinky2": "LeftFootPinky2",
    "LeftFootPinky3": "LeftFootPinky3",
    "RightFootThumb1": "RightFootThumb1",
    "RightFootThumb2": "RightFootThumb2",
    "RightFootThumb3": "RightFootThumb3",
    "RightFootIndex1": "RightFootIndex1",
    "RightFootIndex2": "RightFootIndex2",
    "RightFootIndex3": "RightFootIndex3",
    "RightFootMiddle1": "RightFootMiddle1",
    "RightFootMiddle2": "RightFootMiddle2",
    "RightFootMiddle3": "RightFootMiddle3",
    "RightFootRing1": "RightFootRing1",
    "RightFootRing2": "RightFootRing2",
    "RightFootRing3": "RightFootRing3",
    "RightFootPinky1": "RightFootPinky1",
    "RightFootPinky2": "RightFootPinky2",
    "RightFootPinky3": "RightFootPinky3",
    "LeftUpLegRoll": "LeftUpLegRoll",
    "LeftLegRoll": "LeftLegRoll",
    "RightUpLegRoll": "RightUpLegRoll",
    "RightLegRoll": "RightLegRoll",
    "LeftArmRoll": "LeftArmRoll",
    "LeftForeArmRoll": "LeftForeArmRoll",
    "RightArmRoll": "RightArmRoll",
    "RightForeArmRoll": "RightForeArmRoll",
}

mobuMap_SMPL = {
    "Reference": "reference",
    "Hips": "pelvis",
    "LeftUpLeg": "left_hip",
    "LeftLeg": "left_knee",
    "LeftFoot": "left_ankle",
    "RightUpLeg": "right_hip",
    "RightLeg": "right_knee",
    "RightFoot": "right_ankle",
    "Spine": "spine1",
    "Spine1": "spine2",
    "Spine2": "spine3",
    "Neck": "neck",
    "Head": "head",
    "LeftShoulder": "left_collar",
    "LeftArm": "left_shoulder",
    "LeftForeArm": "left_elbow",
    "LeftHand": "left_wrist",
    "RightShoulder": "right_collar",
    "RightArm": "right_shoulder",
    "RightForeArm": "right_elbow",
    "RightHand": "right_wrist",
    "LeftHandThumb1": "left_thumb1",
    "LeftHandThumb2": "left_thumb2",
    "LeftHandThumb3": "left_thumb3",
    "LeftHandIndex1": "left_index1",
    "LeftHandIndex2": "left_index2",
    "LeftHandIndex3": "left_index3",
    "LeftHandMiddle1": "left_middle1",
    "LeftHandMiddle2": "left_middle2",
    "LeftHandMiddle3": "left_middle3",
    "LeftHandRing1": "left_ring1",
    "LeftHandRing2": "left_ring2",
    "LeftHandRing3": "left_ring3",
    "LeftHandPinky1": "left_pinky1",
    "LeftHandPinky2": "left_pinky2",
    "LeftHandPinky3": "left_pinky3",
    "RightHandThumb1": "right_thumb1",
    "RightHandThumb2": "right_thumb2",
    "RightHandThumb3": "right_thumb3",
    "RightHandIndex1": "right_index1",
    "RightHandIndex2": "right_index2",
    "RightHandIndex3": "right_index3",
    "RightHandMiddle1": "right_middle1",
    "RightHandMiddle2": "right_middle2",
    "RightHandMiddle3": "right_middle3",
    "RightHandRing1": "right_ring1",
    "RightHandRing2": "right_ring2",
    "RightHandRing3": "right_ring3",
    "RightHandPinky1": "right_pinky1",
    "RightHandPinky2": "right_pinky2",
    "RightHandPinky3": "right_pinky3",
    "LeftFootThumb1": "LeftFootThumb1",
    "LeftFootThumb2": "LeftFootThumb2",
    "LeftFootThumb3": "LeftFootThumb3",
    "LeftFootIndex1": "LeftFootIndex1",
    "LeftFootIndex2": "LeftFootIndex2",
    "LeftFootIndex3": "LeftFootIndex3",
    "LeftFootMiddle1": "LeftFootMiddle1",
    "LeftFootMiddle2": "LeftFootMiddle2",
    "LeftFootMiddle3": "LeftFootMiddle3",
    "LeftFootRing1": "LeftFootRing1",
    "LeftFootRing2": "LeftFootRing2",
    "LeftFootRing3": "LeftFootRing3",
    "LeftFootPinky1": "LeftFootPinky1",
    "LeftFootPinky2": "LeftFootPinky2",
    "LeftFootPinky3": "LeftFootPinky3",
    "RightFootThumb1": "RightFootThumb1",
    "RightFootThumb2": "RightFootThumb2",
    "RightFootThumb3": "RightFootThumb3",
    "RightFootIndex1": "RightFootIndex1",
    "RightFootIndex2": "RightFootIndex2",
    "RightFootIndex3": "RightFootIndex3",
    "RightFootMiddle1": "RightFootMiddle1",
    "RightFootMiddle2": "RightFootMiddle2",
    "RightFootMiddle3": "RightFootMiddle3",
    "RightFootRing1": "RightFootRing1",
    "RightFootRing2": "RightFootRing2",
    "RightFootRing3": "RightFootRing3",
    "RightFootPinky1": "RightFootPinky1",
    "RightFootPinky2": "RightFootPinky2",
    "RightFootPinky3": "RightFootPinky3",
    "LeftUpLegRoll": "LeftUpLegRoll",
    "LeftLegRoll": "LeftLegRoll",
    "RightUpLegRoll": "RightUpLegRoll",
    "RightLegRoll": "RightLegRoll",
    "LeftArmRoll": "LeftArmRoll",
    "LeftForeArmRoll": "LeftForeArmRoll",
    "RightArmRoll": "RightArmRoll",
    "RightForeArmRoll": "RightForeArmRoll",
}

# -------------- start utils ---------------
def listFiles(directory, extension=".bvh"):
    results = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                results.append(file)
    return results

def deselectAll():
    modelList = FBModelList()
    FBGetSelectedModels(modelList, None, True)
    for model in modelList:
        model.Selected = False


def addJointToCharacter(characterObject, slot, jointName):
    myJoint = FBFindModelByLabelName(jointName)
    if myJoint:
        proplist = characterObject.PropertyList.Find(slot + "Link")
        proplist.append(myJoint)


def characterizeBiped(namespace, boneMap):
    app = FBApplication()
    characterName = namespace + "bipedCharacter"
    character = FBCharacter(characterName)
    app.CurrentCharacter = character

    # assign Biped to Character Mapping.
    for pslot, pjointName in boneMap.items():
        addJointToCharacter(character, pslot, namespace + pjointName)

    characterized = character.SetCharacterizeOn(True)
    if not characterized:
        print(character.GetCharacterizeError())
    else:
        FBApplication().CurrentCharacter = character
    return character


def plotAnim(char, animChar):
    """
    Receives two characters, sets the input of the first character to the second
    and plot. Return ploted character.
    """
    plotoBla = FBPlotOptions()
    plotoBla.ConstantKeyReducerKeepOneKey = True
    plotoBla.PlotAllTakes = False
    plotoBla.PlotOnFrame = True
    plotoBla.PlotPeriod = FBTime(0, 0, 0, 1)
    plotoBla.PlotTranslationOnRootOnly = True
    plotoBla.PreciseTimeDiscontinuities = True
    # plotoBla.RotationFilterToApply = FBRotationFilter.kFBRotationFilterGimbleKiller
    plotoBla.UseConstantKeyReducer = False
    plotoBla.ConstantKeyReducerKeepOneKey = True
    char.InputCharacter = animChar
    char.InputType = FBCharacterInputType.kFBCharacterInputCharacter
    char.ActiveInput = True
    if not char.PlotAnimation(
        FBCharacterPlotWhere.kFBCharacterPlotOnSkeleton, plotoBla
    ):
        FBMessageBox(
            "Something went wrong",
            "Plot animation returned false, cannot continue",
            "OK",
            None,
            None,
        )
        return False

    return char


def setCurrentTake(desiredName):
    for take in FBSystem().Scene.Takes:
        if take.Name == desiredName:
            FBSystem().CurrentTake = take
            return
    print(f"No take named {desiredName} found")

def is_end_effector(model):
    # Simple check if the model has no children (you can customize this)
    return len(model.Children) == 0

def traverse_and_collect_end_effectors(model, models_to_delete):
    for child in model.Children:
        if is_end_effector(child):
            models_to_delete.append(child)
        else:
            traverse_and_collect_end_effectors(child, models_to_delete)

# -------------- end utils ---------------


source_directory = r"D:\Code\ai\motion_processing\arfriend_data\source"
templat_directory = r"D:\Code\ai\motion_processing\template"
retargetDirectory = r"D:\Code\ai\motion_processing\arfriend_data\retarget"

if not os.path.exists(retargetDirectory):
    os.mkdir(retargetDirectory)

app = FBApplication()
scene = FBSystem().Scene

# sourceMotionFiles = listFiles(sourceMotionDirectory)
# targetSkeletonFiles = listFiles(skeletonsDirectory, ".bvh")

source_target_file_pair = {}
source_files = os.listdir(source_directory)
for source_file in source_files:
    if not source_file.endswith(".bvh"):
        continue
    data_prefix = source_file.split("_")[0]
    source_file_path = os.path.join(source_directory, source_file)
    target_file_path = os.path.join(templat_directory, f"{data_prefix}.bvh")
    source_name = source_file.replace(".bvh", "")
    source_target_file_pair[source_name] = {
        "source": source_file_path,
        "target": target_file_path
    }    



# retarget all source motions to all target skeletons
for source_target_pair in source_target_file_pair.values():
    target_filepath = source_target_pair['target']
    app.FileNew()
    globalBVHCounter = 0
    app.FileImport(target_filepath, False)
    character = characterizeBiped("BVH:", mobuMap_SMPL)
    globalBVHCounter += 1

    deselectAll()
    motionFile = os.path.basename(source_target_pair["source"])
    newTake = FBTake(motionFile)
    scene.Takes.append(newTake)
    setCurrentTake(motionFile)
    newTake.ClearAllProperties(False)
    app.FileImport(source_target_pair["source"], False)
    # set frame rate
    FBPlayerControl().SetTransportFps(FBTimeMode.kFBTimeMode60Frames)
    if globalBVHCounter == 0:
        characterNamespace = "BVH:"
    else:
        characterNamespace = f"BVH {globalBVHCounter}:"
        globalBVHCounter += 1
    motionCharacter = characterizeBiped(characterNamespace, mobuMap)
    # key all frames for bvh to prevent unwanted interpolation between frames
    lEndTime = FBSystem().CurrentTake.LocalTimeSpan.GetStop()
    lEndFrame = FBSystem().CurrentTake.LocalTimeSpan.GetStop().GetFrame()
    lStartFrameTime = FBSystem().CurrentTake.LocalTimeSpan.GetStart()
    lStartFrame = FBSystem().CurrentTake.LocalTimeSpan.GetStart().GetFrame()
    lRange = min(int(lEndFrame) + 1, 50)
    lPlayer = FBPlayerControl()
    for i in range(lRange):
        lPlayer.Goto(FBTime(0, 0, 0, i))
        scene.Evaluate()
        lPlayer.Key()
        scene.Evaluate()
    lPlayer.Goto(FBTime(0, 0, 0, 0))
    plotAnim(character, motionCharacter)
    exportPath = os.path.join(retargetDirectory, motionFile)
    print(f"Retarget motion {motionFile} to {exportPath}")
    character.SelectModels(True, True, True, True)
    app.FileExport(exportPath)
    deselectAll()