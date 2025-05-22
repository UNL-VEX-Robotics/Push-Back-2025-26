# Push Back
This repository contains all of the code used *directly on robots* for the 2025-26 VEX season. The repositor is split into branches by robot.

## Branches
* RI3D - Our robot in 3 days for preseason.

## Dependencies
### bash
For dependencies below, you will need to use bash to set up some merge settings. This will help to prevent merge errors so you will not have to manually resolve them.
```bash
git config merge.ours.driver true
git config merge.theirs.driver 'git checkout --theirs -- %A'
```
### neblib
This repository is dependent on the SKERS neblib.(https://github.com/UNL-VEX-Robotics/neblib)  
To update neblib within the repository, use git bash:
```bash
cd /path/to/current_repository
git remote add neblib https://github.com/UNL-VEX-Robotics/neblib.git  
git fetch neblib
git merge neblib/main
```
Ensure that you are in the main branch of the Push Back repository.

From here, check out the branch of the robot you are currently working on. Using bash:
```bash
git checkout <your-branch>
git fetch origin
git rebase origin/main
```
Alternatively, if you are certain the main branch on your device is up to date:
```bash
git checkout <your-branch>
git rebase main
```
To update all branches simultaneously, use this loop: 
```bash
for b in $(git branch --format='%(refname:short)' | grep -v '^main$'); do
  echo "Merging into branch: $b"
  git checkout "$b" && git merge main
done
```
