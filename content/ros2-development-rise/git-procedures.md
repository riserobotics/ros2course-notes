+++
title = "Git procedures"
weight = 1
+++

*This is adapted from the text in the main code documentation [here]() and visible as part of *

Every development of code within the RISE project has to happen under `git` based version control. We want to be very clear here: except for initial testing and trialing, every bit of source code that is intended to at some point end up in productive use **has to happen in a git based repository**. Code that is submitted in any other form, be it zip archives, links to Nextcloud storage directories or USB thumb drives will be considered as **not submitted**. There are very few examples of projects that have been able to organize development efforts successfully with no involvement of a standardized version control system. The ones that have been able to do so, are usually led by quite senior people who have a lot of experience in developing software collaboratively. Therefore, knowing how to use git within the context of this project is important!  

{{% notice tip %}}
This applies to all **source code**, i.e. textual data within the project. We also version control other types of files with git and on GitHub, for example PCB layouts, these obviously have to follow other procedures that are documented within their respective wikis. 
{{% /notice %}}

There are several common procedures for how to use `git` effectively when collaborating on a code base within a team. They all have their advantages and disadvantages, and you may have a personal preference to use a different procedure. Even if, within your group, you agree that a different methodology is better, **please don't deviate from the process described here**. The code that is written here should ideally be maintainable for many semesters after you (potentially :) )depart from the project, so keeping consistency is important!

#### Trunk based development
All of our git repositories have a main branch, called [main](https://sfconservancy.org/news/2020/jun/23/gitbranchname/). This branch has several features, that make it different from other branches on the project. Some of them apply only to selected repositories, that contain more complex codebases. 

1. The branch is protected, meaning it can not be deleted or edited by users with no owner privileges. This is to prevent accidental changes to the repository structure, and make sure that the branch will always continue to exist. 

2. You are not able to commit to the main branch. If you clone the repository, make changes to the main branch and then try to commit, GitHub will reject your attempt. This is to make sure that all changes to this branch have gone through a pull request (PR) process before being integrated. It's not allowed to force push either. This rule also applies to Maintainers. 

3. There are workflows that test the code. Whenever a pull request is made that targets the main branch, a set of workflows will run testing if, for example linting guidelines have been followed and if the project builds. 

#### Planning & Organization
We strive to organize all working packages that need to be done, as part of the project within Git Issues. To see a tutorial on how to use and open up Issues see [here](../git/git-issues/). You should understand issues as a description of a work package, they can also be understood as a task description. The issue will contain all the information needed, to code up a working solution to the problem described within. 

Issues might also be so called "Epics", in that case they serve as a summary of multiple issues that are linked inside. Epics should be used for larger features and operations, so most Issues in this project will probably be standalone. In case you are working on an Issue that is classified as an Epic, make sure to read the description carefully, as some parts of that issues might be blocked by others, i.e. they require the other Issue to be merged first

#### Getting assigned to an Issue. 
There are multiple ways on how you can get assigned as an issue.

- You may get assigned to that issue by the author, who thinks you are a good fit for wroking on it. In that case, feel free to contact the person who assigned you to get more information. If you are looking for information on the specific requirements of an Issues, contact the author. 

- You assign yourself for an issue. If you see an open issue that is not assigned to someone you can claim that issue, if you don't see a [WIP!] designator in the title. If that is the case, then the issues is not yet finished and still needs additional information which has to be submitted by the author. Before you start working on that issues, make sure to communicate with the author about potential context information. The issue should contain all content relevant for solving, so usually you should not get any more information, however it is good practice to still check. 

If you see an issue that is assigned to someone, however there has not been any activity for a while and you think you can contribute, write a comment in the issue log and ask if help is needed and you can take over.

#### Working on an Issue
If you have an Issue assigned to yourself, you may start working on that issue. First step is to add the Label "Doing" in the "Labels" section of the issue. This lets other people know that the issues is actively worked o, and moves the card in the kanban board that we use for visualizing project progress. 

Next, you have to create a branch for the issue if it requires changes to the codebase. For a How-To guide on creating a new branch see [this](../git/git-branches/) tutorial. If the issue does not require a change to the codebase, i.e. it has the Labels "Admin" or similar, no branch has to be created. After you created a branch, it has to be checked out on your local machine, this process is described in [this](../git/git-checkout/) tutorial. 

While working on the issue keep all relevant [coding guidelines](../../guidelines/) in mind, to be sure that your contributions can be accepted without lengthy review. In case additional standards or requirements on the specific implementation on a solution are needed, e.g. timing or similar, they have to be specified in the issue.

During your time working on the issue you should commit your results frequently. For a detailed description on how to space individual commits, see [this](../git/git-commits/) tutorial. Make sure to follow the recommendations on writing good commit messages, so tht others can follow the contributions you made. 

#### Documenting your issue.
Providing code without documentation is deemed insufficient and can not be merged. We will **not** accept issues without documentation, period. In case your issues is concerning a major update, there will most likely be a linked issue in the [documentation repository](https://git.tu-berlin.de/rise/riseos-docs) already created. In order to merge your merge request, the documentation issue has to be accepted first. In case your issue did not feature a documentation issue, you will have to create an issue in the documentation repository yourself. 

#### Finishing up a completed issue
If you are done with your work on an issue, and all the requirements are fulfilled you should test your solution thoroughly. The issue itself will specify a list of acceptance criterias, please try to verify that these requirements are met before proceeding further. This makes reviewing and testing easier for others down the line, and keeps everyone happy. A well tested merge request can be handled more quickly. 

Once you feel confident that your merge request fulfills the criteria, you can create a merge request to merge your changes into the main branch. See [this](../git/git-merge/) tutorial for creating a merge request. Unless you are a Maintainer yourself, you will not be able to merge the merge request yourself. Every merge request has to be reviews first. For commits to the main repository, two reviews are suggested, with one of the two reviewers being a Maintainer. For the other reviewer feel free to select any one of your teammates. Reviewing merge requests and testing them is good practice to understand the whole codebase, and improving your ability to judge code for completeness and quality. It also distributes the testing process over more shoulders and makes sure that errors can be spotted quickly and reliably. 

Your merge request will now be reviewed. There are three possible outcomes.

- There are requested changes: The reviewer requests changes if he feels like something has to be changed. Take a look at the comments he or she left you and try to understand the intention. If you feel like something is misunderstood, contact the reviewer in the comments of the merge request. Try to understand where the error is coming from. In the end, the software should launch on every system, so if something did not work on the system of the reviewer, it's likely still your fault to not provide enough documentation od reliability in code. Adress the changes the reviewer noted in additional commits to the same branch. If you are done, and feel like all issues have been adressed, request a review again. 

- There are no requested changes: If that's the case, and both reviews return favorably, your merge request is merged and will be visible in the main branch of the repository. 

- There are no requested changes to the code, but your documentation is insufficient: The reviewer did not accept your issue to be merged, however there are no requested changes to the code. Please address his concerns in the corresponding issue in the documentation repository. 

> Please make sure to contact the reviewer **only** with the comments within the merge request, and **not** with any other communication channel. If someone else is reading back on that merge request later, he will be confused if the communication is not transparently visible and lost in some other communication channel


