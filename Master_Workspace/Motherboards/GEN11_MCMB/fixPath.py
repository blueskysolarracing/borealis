def fixLine(lineToChange)->str:

    thisFilePath = __file__
    thisFilePath = thisFilePath.replace("\\", "/")

    index = thisFilePath.rfind("/gen11_blueskyelec") + len("/gen11_blueskyelec")
    if index == -1:
        print("Error: Please make sure your top repository is gen11_blueskyelec")
        quit()

    topRepoPath = thisFilePath[0:index].replace("c:/", "C:/")
    
    beginIdx = lineToChange.find("<location>") + len("<location>")
    if beginIdx == -1:
        print("Error: <location> not found in file")
        quit()
    beginStr = lineToChange[0:beginIdx]

    replaceToIdx = lineToChange.find("/Master_Workspace")
    if replaceToIdx == -1:
        print("Error: /Master_Workspace not found in file")
        quit()
    endStr = lineToChange[replaceToIdx:]

    newLine = beginStr + topRepoPath + endStr
    return newLine

def main():
    projectFile = open('.project', 'r')
    fileLines = projectFile.readlines()
    projectFile.close()
    projectFile = open('.project', 'w')
    i = 0;
    while i < len(fileLines):
        if "/Shared_Resources" in fileLines[i]:
            projectFile.write(fixLine(fileLines[i]))
        else: 
            projectFile.write(fileLines[i])
        i += 1;
if __name__ == "__main__":
    main()