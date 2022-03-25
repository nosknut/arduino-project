import os
import json

workspace_indicator_file = "platformio.ini"
workspace_config_file = os.path.basename(os.getcwd()) + '.code-workspace'

# recursively dives through directories looking for workplaceFile


def find_workspaces(parent_path):
    # print the absolute path of parent_path
    if os.path.isfile(os.path.join(parent_path, workspace_indicator_file)):
        return [parent_path]

    if os.path.isdir(parent_path):
        paths = []
        for filename in os.listdir(parent_path):
            filepath = os.path.join(parent_path, filename)
            if os.path.isdir(filepath):
                childPaths = find_workspaces(filepath)
                if childPaths:
                    paths += childPaths
        return paths

with open(workspace_config_file, 'r') as f:
    jsonFile = json.load(f)
    folders = jsonFile.get("folders")
    existing_paths = {folder.get("path"): folder for folder in folders}
    for path in find_workspaces("./"):
        if path in existing_paths:
            folders.remove(existing_paths[path])
            print("Updating", path)
        else:
            print("Adding", path, " to workspace")
        folders.append({
            "path": path,
            # Option one:
            # os.path.dirname(os.path.normpath(path)) + '-' + os.path.basename(path)
            # subFolder-workFolder
            #
            # Option two:
            # '/'.join([os.path.basename(os.getcwd()), *os.path.split(path[2:])])
            # projectFolder-subFolder-workFolder
            "name": '/'.join([os.path.basename(os.getcwd()), *os.path.split(path[2:])])
            })
    
with open(workspace_config_file, 'w') as f:
    f.write(json.dumps(jsonFile, indent=4, sort_keys=True))
    