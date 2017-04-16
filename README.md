
# AuE893F1tenthProject

This is a common repository to collect all the working code and note files developed for the F1tenth car.
How to use GitHub to submit code while logged into your own GitHub account:

1.    Fork the repository.
2.    Make the change.
3.    Submit a pull request to the project owner.


    
**Fork the repository to add changes to the code stored here.**

Navigate to this repository and in the top right corner click "Fork"
Now you have this repository reflected in your own Github account. 

Create a directory on your system to house the code you pull and push from here.
cd into that directory and run: git clone https://github.com/YOUR_USERNAME/AuE893F1tenthProject

Now suppose changes have been made to the master repository since the time you have cloned it. 
You can configure Git to pull changes from the original, or upstream, repository 
into the local clone of your fork.

This needs to be done only once:

cd into the directory you created. 
run: `git remote -v` (this shows you your clone of this repository. You will add changes to this)
  `
origin  https://github.com/YOUR_USERNAME/AuE893F1tenthProject.git (fetch)

origin  https://github.com/YOUR_USERNAME/AuE893F1tenthProject.git (push)
  `

run: git remote add upstream https://github.com/araman92/AuE893F1tenthProject (this allows you to integrate
changes in this repository with yours)

To verify the new upstream repository you've specified for your fork, type `git remote -v` again. You should see the URL for your fork as origin, and the URL for the original repository as upstream.
  
  `
git remote -v

origin    https://github.com/YOUR_USERNAME/AuE893F1tenthProject.git (fetch)

origin    https://github.com/YOUR_USERNAME/AuE893F1tenthProject.git (push)

upstream  https://github.com/araman92/AuE893F1tenthProject (fetch)

upstream  https://github.com/araman92/AuE893F1tenthProject (push)

  `

Now, if changes are made in the future and you want to sync your fork with the main repository again:

git fetch upstream (this will create a new branch called upstream/master in your repository)

git checkout master (this will send you into your own master branch)

(you can now add changes to this master branch)

** Short recap on how to add changes:**
      `
git add . (your file)
git commit -m "message about the update"
git push origin YOUR_BRANCH (unless you created a new branch this is the default branch: master)
`
login with your id/password

git merge upstream/master (this will merge your master branch with the upstream/master branch so you don't lose the changes you made in your own master branch)

** Navigate to your master branch on your repository on the webpage and submit a pull request. **



