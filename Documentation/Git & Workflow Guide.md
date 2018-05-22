Git & Workflow Guide
====================

Creation date: Summer 2016
Updating date: 22.05.2018

## I. Coding Workflow

This chapter starts with an overview of the coding workflow used in the Åland Sailing Robots project and then provides some examples that are in the context of this project. The used coding workflow is based on the feature branch and gitflow workflows. They are presented here: https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow 

**1.1 Overview**

A repository has two main branches - master and develop - and other branches known as feature branches. The master branch contains tested code that works and have been tested on the boat. Any bugs in it should be minor. The develop branch contains code that compiles and have been tested land side but needs to be tested on the boat in the water.

Before the boat is taken out to test a new feature, a field_test branch is created from the develop branch. The code inside this branch is the one running during the test. It can be seen as a release branch - only bug fixes, documentation generation, and field-oriented tasks should be done in this branch. If the test is conclusive, then this branch gets merged into master and tagged with a version number. The branch should also be merged back into develop to take into account any work carried out in the test_field branch.

Feature branches are used to develop new features/topics or to fix bugs. Each new feature resides in its own branch, which can be pushed to the GitHub remote repository for backup and collaboration. Each feature should have a unit-tests suite that allows for easily testing the code. Once a feature and its associated unit-test suite are considered complete – the code is clean and commented, it compiles and all unit-tests pass – a pull request is send on GitHub for code reviewing. This step is essential before the code is merged into the develop branch. The GitHub pull request tool is ideal for code reviewing, allowing code comparison, comment and suggestions. The review of the code doesn’t require the use of this tool but has to be done by another team member before merging any feature branch into the develop branch.

Archive branches can also be found. They correspond to old unused code that we want to keep as a base for future developments.

**1.2 Examples**

In this example we will look at two features - WaypointManager and SystemLogger - being developed by two separate people - Mary and Ben. 
Mary is working on a new waypoint manager and ben is working on a new system logger. Both of them may touch similar code areas to ensure their features work correctly. If they were working on the same code branch this could lead to a lot of code merge conflicts when they commit code. So to avoid this problem they use the Gitflow workflow. 

-> They start with a stable branch called master and development branch called develop. At the start the development branch is identical to the stable branch.

-> Mary begins work on her assigned feature and creates a new branch based on the branch called master. She uses the following command:

	```console
    # git checkout develop**     				// switches her working branch to develop
    # git checkout -b waypoint_manager 			// creates a new branch called waypoint_manager, based on her current working branch which is the branch develop.
	```

-> Mary makes two code commits on her branch using the commands git add and  git commit. See “II. Git introduction & common commands” for more informations on those commands.

	```console
	# git status
	# git add <some-file>
	# git commit
	```

-> Ben creates a new branch for his feature using the same commands Mary did and then makes a single code commit.

-> Mary decides she is happy with her code  – the code is clean and commented, it compiles and all unit-tests pass. She needs to file a pull request on GitHub for code reviewing.

	```console
	# git push origin waypoint_manager 			// Push her branch to the remote repository
	```

Create a pull request into GitHub

-> Ben gets the pull request and takes a look at Mary’s-feature. He decides he wants to make a few changes before integrating it into the official project, and he and Mary have some back-and-forth via the pull request.

-> Once the pull request is accepted, mary will merge the remote develop branch into the waypoint_manager branch.

	```console
	# git checkout develop 						// Switches to the develop branch
	# git pull origin develop 					// Pull the latest version of the develop branch
	# git checkout waypoint_manager				// Switches back to the feature branch
	# git merge develope 						// Merge develop into waypoint_manager
	```

This is good practice to first merge the develop branch into the feature branch. Any merge conflicts will come up here and should be fixed before merging the code back into develop.
The unit-tests are run again at this stage and all have to pass before going to the next step.
Changes from the merge with develop are pushed to the remote branch:

	```console
	# git push origin waypoint_manager			// Push merge changes to remote feature branch
	```


-> At this point Mary’s branch (waypoint_manager) has her code in it and any other code that was put in the develop branch after she created her branch. She will now merge her branch into develop. As nothing else has been put in her branch since the merging, it shouldn’t have any merge conflicts.

	```console
	# git checkout develope 					// Switches her working branch into develop branch
	# git merge waypoint_manager				// Merge waypoint_manager branch into develop
	# git push origin develope 					// Push the updated develop
	# git branch -d waypoint_manager			// Delete the local feature branch
	# git push origin --delete waypoint_manager // Delete the remote feature branch
	```

-> Ben makes one more commit and then decides his code is ready for the boat. So he adds his commit, pushes his code to the remote repository and creates a pull request. After it has been accepted, he can merge the develop branch into his branch so he will have all the code Mary has written.

-> Mary and Ben edited similar lines of code so when Ben merged the develop branch into his branch he had some merge conflicts. After fixing them and make sure that all unit-tests pass, he merges his feature branch (system_logger) back into develop.

-> At this point both of the new features are in the develop branch. We want to test those new feature on the boat. A field_test branch is created from the develop branch and will run the code during the tests on the sea. 

-> After some testing on the boat and some bug fix, these two new features are seen as working and complete so the field_test branch is merged into the stable branch, master. A tag with a version number is add to the merge commit of the master branch. The field_test branch is also merged back into develop to take into account the changes made to fix the bugs.

## II. Git introduction & common commands

Git is a version control system for tracking changes in computer files and coordinating work on those files among multiple people.
GitHub is an online work tool that allows you to share changes made by several people on a given project.
The most common way of using git with GitHub is to have a remote repository hosted on GitHub and local copies of this repository on each developer computer. That allows developer to work off-line and return its changes later to the online project.

**Setting up a git environment**

Configure user information for all local repositories :

	```console
	# git config --global user.name <name>
	# git config --global user.email <email address>
	```

Git uses a username to associate commits with an identity. The Git username is not the same as your GitHub username.

**Setting up a git repository**

Initialize a new local repository :

	```console
	# git init [repository_name
	```

Connect local repository to remote repository :

	```console
	# git remote add origin https://github.com/username/new_repo
	```

Clone an existing repository :
	
	```console
	# git clone [url]								// ex: git clone https://github.com/AlandSailingRobots/sailingrobot.git
	```

**Saving changes locally**

Developing a project revolves around the basic edit/stage/commit pattern. First, you edit your files in the working directory. When you’re ready to save a copy of the current state of the project, you stage changes with git add. After you’re happy with the staged snapshot, you commit it to the project history with git commit. A commit should be done after having : adding a new piece of data or function; fixing one or more bugs; refactoring code or data...

Save code changes is simple, first it is good to see what code has actually changed since the last commit. Display the state of the working directory and the staging area :

	```console
	# git status									// List which files are staged, unstaged and untracked by git
	```

Add changes to the staging area (Index):

	```console
	# git add <path of file>        				// Stage all changes in <file> for the next commit
	```

Remove <file> from the staging area, but leave the working directory unchanged :

	```console
	# git reset <path of file>
	```

Commit the staged snapshot :

	```console
	# git commit -m “type a message about what you committed in these quotes”
	```

**Syncing with the remote repository**

The commands presented below let you manage connections with other repositories, publish local history by "pushing" branches to other repositories, and see what others have contributed by "pulling" branches into your local repository.

To update local repository with all remote changes :

	```console
	# git pull 										// = fetch + merge
	```

To upload all the git commits in your branch to remote repository (origin) :

	```console
	# git push origin <branch_name>
	```

**git Branches**

A branch is a space in which the work done will not modify the file in the other branches. A branch allows to develop its work in a clean and isolated container. Branches can be merged to share changes made to files.

To switch the working branch :

	```console
	# git checkout <name of branch>
	```

To create a new branch :

	```console
	# git checkout -b <name of branch>
	```

To delete branch :

	```console
	# git branch -d <branch_name>            		// local deletion
    # git push origin --delete <branch_name>        // remote deletion
	```

To merge one branch into another branch you need to checkout the branch you want the code to be merged into then run the following command:

	```console
	# git merge <name of the branch you wish to merge into the working branch>
	```

If no conflicts appear, the branches can be merged automatically. Otherwise, conflicts must be resolved by hand.

**TODO: ADD MERGE CONFLICT**

**Pull Request**

A pull request is a useful tool for other members of your team to review and comment on the changes in your branch before a merge. 
To create a pull request, simply log into GitHub and go to the code repository. From there, click on “New Pull Request”, and choose branches. The “base” branch is the branch you want to merge into, for example; develop. The “compare” branch is your branch that you have been working on, and is the branch that you want to merge. From there you can add your initial comment, and see a list of commits and changed files, and then simply click the big green button, “Create Pull Request”. From then on, other members on the team can add their own comments on the commits. 

**Restore**

	```console
	# git checkout --<filename>    					// replace working copy with latest commit
	```

**Inspecting the project history**

Follow, inspect and compare the evolution of project files :

	```console 
	# git log --graph --online --decorate    		// get a nice overview of the project history
	# git log --follow <filename>
	# git diff <source_branch> <target_branch>      // view changes between 2 branches
	```

**Tagging**

	```console
	# git tag -a <tag> -m “tag message”    			// Create tag on the last commit
    # git push --tags            					// Push tags to remote repository
    ``` 

**Submodule**

A submodule allows you to keep another Git repository in a subdirectory of your repository. The other repository has its own history, which does not interfere with the history of the current repository. This can be used to have external dependencies such as third party libraries for example.

Inspects, updates and manages submodules :

	```console
	# git submodule init     						// initialises all the submodules (Only need to do this once)
	# git submodule update    						// Updates the submodule to the latest version
	```

To know the good practice to deal with submodules : https://delicious-insights.com/en/posts/mastering-git-submodules/

**Gitignore**

Git sees every file in your working copy as one of three things:
1. tracked - a file which has been previously staged or committed;
2. untracked - a file which has not been staged or committed; or
3. ignored - a file which Git has been explicitly told to ignore.

Ignored files are usually build artifacts and machine generated files that can be derived from your repository source or should otherwise not be committed. 
Ignored files are tracked in a special file named .gitignore that is checked in at the root of your repository. There is no explicit git ignore command: instead the .gitignore file must be edited and committed by hand when you have new files that you wish to ignore.
