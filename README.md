# COMP3631 Group Project Repository
This repository holds all required files and documentation for the COMP3631 group project.

See below some details and guidelines about contributing and other useful details.

# Contributing
In order to make sure that work does not get overwritten, that there are no conflicts and that breaking changes can easily be located, contributions to the repo should follow a simple set of rules.

**In short, each group member will do their work on a separate git branch locally, and then that local branch will be pushed to the remote repo. A pull request will be requested to merge the changes on the branch into the `main` branch, and if all is well, the changes will be merged.**

To illustrate this, follow the example below which describes how to integrate a change in the motion logic into the `main` branch.

Let's say that group member X needs to implement a change that allows the robot to go into the room located by the red circle:

0. Make sure you are in the `group_project/` folder and on the `main` branch (`git status` to check the branch you are on)
1. Run `git pull` to pull all recent changes from the upstream `main` into your local `main` branch
2. Run `git checkout -b X/move-to-red-room` to create a new branch named `X/move-to-red-room` and checkout (move) to it
3. Confirm you are on the `X/move-to-red-room` branch by running the status command from step 0
4. Develop the changes
5. Git add, git commit, git push
6. Make sure the changes are on the correct remote branch (check the GitHub UI branches section on the repo)
7. Open up a Pull Request from `X/move-to-red-room` into `main`
8. Allow group members to reveiw & test the changes if necessary
9. Merge

# Useful Links
1. Shared Group 18 Progress Report Document: [link](https://leeds365-my.sharepoint.com/:w:/g/personal/sc19kk_leeds_ac_uk/EZTKU-N0UalGgQdTB2NNFxgBpY9ET-WmHQG6-PznJdWeWQ?e=8UuVFj)
2. Plan: [link](https://docs.google.com/document/d/1v1eqLz01KpyRcKHfHGMsqp5kTWpoTyFIsMvJrJAQAVQ/edit)
3. Shared Final Report Document: [link](https://leeds365-my.sharepoint.com/:w:/g/personal/sc19kk_leeds_ac_uk/Eeu7eez3539Bs9FGgH8Z1d8BVL_ceM3Ql5siwvCdVX6SIQ?e=zOfgaDO)