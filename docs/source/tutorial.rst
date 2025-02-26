Tutorial
=============

**Section 3 on rj_gameplay and Python is outdated as of 12/28/2022.**
(The other sections still apply.)

This page is meant to teach new RoboCup Software members the basics of what
they'll need to contribute. It will introduce the Robot Operating System (ROS),
the command-line, GitHub, git, Python, C++, and the general shape of our stack.
Upon completion of this tutorial, you'll have the knowledge and trust of your
teammates to implement new features on your own.

No prior experience is assumed. However, a bit of stubbornness is required.
RoboCup SW has seen many members without any prior CS experience become valued
contributors, and many talented CS majors quit within a few weeks.

**Note: you must have Ubuntu 20.04 installed in some capacity to use our
stack.** Most new members do this with a VM, which is the quickest way. Google
"how to install Ubuntu VM on <your current OS>" and follow a tutorial online.
If you decide to stick with RoboCup long-term, it might be worth your time to
invest in a used laptop that only has Ubuntu 20.04 and your RoboCup files
installed.

The tutorial is structured as follows.

.. contents::

There are some gaps intentionally left in the tutorial. This is to force you to
problem-solve on your own, simulating what it feels like to write a new feature.
If the tutorial was simply a bulleted list of commands to type, it would not
prove that you're ready to work on something meaningful on your own.

When you run into issues, your order of question-asking should be:

#. Google

   * Keywords, not full sentences

   * Error messages, if they come up

#. Google

   * Seriously

#. FAQ page in our docs (common errors and debug info)

#. Fellow new members

#. Software lead

#. Anyone the SW lead takes advice from

This is not because older members don't want to help you, but because if older
members helped every new member with every question, they wouldn't have time to
make our robots better (nor would you learn as much). So try to resolve your
issue yourself, and expect to be asked "what have you tried already?" when you
ask for help.

0. Command-Line Basics
----------------------

If you've never heard of or used the command-line before, `this website
<https://ubuntu.com/tutorials/command-line-for-beginners#1-overview>`_ is
wonderful for beginners.

.. image::

   ./_static/ubuntu_cli_tutorial.png

The rest of this tutorial assumes you have working knowledge of the
command-line: how to run an executable, change directories, move files, run
commands, etc. So if you're uncomfortable with any of that, go through the
exercises in the site above.

Some tips about learning how to use commands:
 * ``man [command]`` will pull up a manpage, which is an explanation of the
   command and all of its options. This usually only works on standard Unix
   commands. For instance, you can find words in any file in a directory using
   ``grep``: try ``man grep`` to see its full potential.
 * ``[command or executable] --help`` will almost always return a prompt that
   tells you what the command does, and how you can modify it with options. Many
   custom command-line tools will have a --help output, if they don't have a man
   page.
 * Of course, you can also simply Google a command you don't understand, or look
   up something like "how to search for a filename with command line".

1. Installation
---------------

See "Installation". That page will assume you have the Command-Line Basics
from above, as well as a working knowledge of Git (which you can get either
`online <https://rogerdudler.github.io/git-guide/>`_ or from the "Contributing"
page).

2. GitHub Basics
----------------

Now that you have everything installed and understand the basics of the
command-line and git, let's get started using GitHub.

.. Note::

   git is a command line version-control tool. GitHub is a website to host
   shared files, and is well-integrated with git, but is not the same thing.

First, use git to create a new branch under this naming scheme:

.. code-block:: bash

   git checkout -b "<your-name>/robocup-sw-tutorial"

For instance, the author's branch would be named
``kevin-fu/robocup-sw-tutorial``.

Launch soccer (our UI) and the
ER-force simulator, same way as you did in the installation guide, then select
this play as the test play to see it in action. 

Now that you've made a change to the repo, run ``git status``. You should see
that whatever files you changed show up in red, which indicates that they are
unstaged. Stage the files you changed with ``git add`` (Google this if unsure
how, or see the previous section on git), then commit them:

.. code-block:: bash

   git commit -m '<commit msg>'

.. note::

   <commit msg> should be a present-tense description of what you've changed. In
   this case, "change to 4 wallers" is fine.

   Without the -m flag, git commit will open a nano, a text editor, and ask you
   to type in a commit msg. -m is a bit faster.

When you commit, you should see our pre-commit hooks run. These are automated
programs that make your code comply with standardized style guidelines. If one
of the checks fails, simply re-add your files and re-commit. (If you don't see
this, make sure you have everything installed correctly per the installation
guide.)

Now that you've committed, run ``git push`` to push your changes to the remote
server. This is how GitHub sees your changes. If you run into any errors at this
step, read the error logs carefully (they often tell you what to do), and Google
if needed.

Finally, go to our GitHub page, click the "Pull Requests" tab, and create a new
draft pull request for your branch. When it asks you to fill in the PR
description, you can delete the template and write something simple like
"Completes RC SW tutorials." Add that screenshot of your four-waller setup as a
comment below your brand new PR. Nice work!

1. rj_gameplay and Python
-------------------------

In this section, you'll be tasked with creating a new Python class to give our
robots some new tricks on the field. This section is one of two coding-heavy
sections, and should present a significant challenge.

 * If you don't know Python, but you've coded in some other language before,
   Python is likely an easier language to learn than the one you already know.
   (Just look at some of the .py files in this repo and you'll see.)
 * If you've never coded before this club, hopefully you are in CS 1301/1371,
   and you'll start learning how to code very shortly. In that case, skip to
   section #4 for now, continue working, and come back here at the end of
   section #5 when it becomes necessary to have this section done.
 * If you've never coded before and you're not in an introductory CS course,
   you'll have to go through a Python tutorial like `this one
   <https://docs.python.org/3/tutorial/>`_ to learn the ropes.

Your task is to create a Runner Role that can make any arbitrary robot run
around the perimeter of the field. This should hopefully distract the other team
and keep them from being able to score on us. **Read the rest of this section
before starting.**

The coordinates of the field are in the ``world_state`` object that is passed
through every single gameplay element. Search the ``rj_gameplay`` folder for
``world_state.field`` to figure out how to get those coordinates--you will
eventually find the file where the field coords are passed in, and from there it
will be obvious how to use them. Do this search with ``grep``, not by hand.

A Role defines a complex, single-robot behavior, like the Goalie, or a Passer.
See the Design Docs linked in `this PR
<https://github.com/RoboJackets/robocup-software/pull/1811>`_ for more detail.
The superclass for all Roles is defined in ``rj_gameplay/stp/role/__init__.py``,
and the subclasses that define actual Roles are in
``rj_gameplay/rj_gameplay/role/``. All roles use a finite state machine, or FSM,
which is really just a good mental model for writing programs that change over
the course of time. Look at the existing files to figure out how to structure
and implement your role to use an FSM.

If you've never heard of a superclass before, see `this website
<https://www.whitman.edu/mathematics/java_tutorial/java/objects/inheritance.html>`_
for a quick introduction. If you want to learn more about FSMs, see `this link
<https://flaviocopes.com/finite-state-machines/>`_.

To test your Role, you'll have to write a Tactic that uses it, and put that
Tactic into a Play. This is a little complicated. Look at the Defense Play you
modified earlier. The Goalie Tactic in this play is really just a wrapper for
the Goalie Role (as in, it doesn't do much but call the Goalie Role and ask it
what to do). This is how your Runner Role should be included. Put it in the
Defense Play so that you have 4 Wallers, 1 Goalie, and 1 Runner. Remember to
test often with the sim.

.. image::

   ./_static/basic_defense.drawio.png

There are many ways to assign a Role to a given robot (see the design doc linked
above for more detail). In this case, assign robot 1 to be our runner. (That's
our most in-shape robot, so it can handle the extra miles.) Do this the same
way that the Goalie Role always picks robot 0 to be the goalie.

You may have noticed there's a lot of file-finding in this section. Use the
option in your IDE or text editor that allows you to see a full folder at once.
For instance, in VS Code, there is an option to open a full folder, which
displays all the subfolders and files in the left toolbar. If you open
``robocup-software/rj_gameplay`` like this, it should be a lot easier to
navigate these files.

If you've read this whole section and are feeling a little intimidated, that's
normal. The paragraphs above form a nice to-do list for you to follow. Just try
your best, one step at a time, and eventually you'll have a working piece of
software to be proud of. You'll use this same Runner Role again later on, so
you'll get to savor your success then!

4. ROS CLI Basics
-----------------

This section is our variation of the ROS 2 `"Beginner: CLI Tools"
<https://docs.ros.org/en/foxy/Tutorials.html#beginner-cli-tools>`_ tutorials. We
do things slightly differently (and don't use all of the ROS 2 features
described in those tutorials), so this is intended to keep you from having to
read all of those docs.

However, those docs are obviously still the source of truth on ROS. Before we
get started, read all of the short "Background" sections for these pages:
 * Understanding ROS 2 nodes
 * Understanding ROS 2 topics
 * Understanding ROS 2 services
 * Understanding ROS 2 parameters
 * Understanding ROS 2 actions

The background sections put together are only a couple hundred words, and
contain very neat animated diagrams that we can't recreate here.

Now that you have some background on what ROS is and how it works, let's explore
how we use ROS in our stack. (ROS is used in place of ROS 2 in the rest of these
docs, just know that we are referencing ROS 2 every time.)

First, open up our stack, same as you did in the installation guide. (Remember
to source ROS2!) Then run

.. code-block::

   ros2 topic list

to see the list of topics. Let's look at what robot 0 is thinking. Run

.. code-block::

   ros2 topic echo /gameplay/robot_intent/robot_0

to see what's being published to that topic. You should see that robot 0 is
being given a motion_command to go to a certain position at a certain angle.
Feel free to try echoing other topics to see what they're publishing.

Now run ``ros2 topic info`` on the same topic to see what message type that
topic is publishing, and how many publishers and subscribers are listening to
it. For this topic, the message type is a subset of ``rj_msgs/``, which means we
wrote our own custom .msg file that this topic uses.

Your task for this section is to find the file that defines the message type
used by ``/gameplay/robot_intent/robot_0``. This will take you a long time if
you search for it manually and almost no time if you use a tool like ``find``.
Once you have the right file, figure out the full filepath and add it to your
GitHub PR as a comment. Congrats! You now have a grasp of ROS CLI tools.

5. rqt Basics
-------------

The observant among you may have noticed that the last section only covered ROS
topics, even though it asked you to read about ROS nodes, services, parameters,
and actions as well. This was to set up the need to use ``rqt``, a graphical
interface for the many tools ROS includes.

To use it, open a new terminal, source ROS (like you do before running our
stack), and run ``rqt``. (This should have been installed with the rest of the
stack when you ran ``./util/ubuntu-setup``; if not, see `this guide
<http://wiki.ros.org/rqt/UserGuide/Install/Groovy>`_.) You should see a blank
GUI pop up.

.. image::

   ./_static/blank_rqt.png

To replicate what we did in the last section, go to the top, click Plugins >
Topics > Topic Monitor. This allows you to see both a list of all topics, and
see the most recent message published to any topic (by clicking the checkbox).

Now find and launch the Node Graph. You should see a large, complex node diagram
pop up. If you don't see something large and complex, make sure you have both
our AI and the ER-Force simulator running.

Zoom in on the Node Graph. You should notice and most of the nodes are actually
just duplicated across robot numbers. (For instance, notice there is a
``/planning/trajectory/robot_*`` topic for each robot.) Find the two arrows that
are labelled with robot 0's robot intent and figure out which nodes publish and
subscribe to that topic. Post your answer as a GitHub comment on your PR.
(Hint: There are **two** nodes that subscribe to this topic.)

We can also use rqt to dynamically change the behavior of our robots. Pull up
the Dynamic Reconfigure menu and click the control params. Run your runner play
from earlier. In the middle of the play, double the max velocity. You should see
the runner (and every other robot on our team) move much more quickly.

Take a screen recording of this whole process and send it to your software lead
via Slack. Feel free to play around with any other params you see!

6. ROS and C++
--------------

Much like the last section, this section is our version of an official ROS
tutorial. This time we'll reprise `Writing a simple publisher and subscriber
(C++)
<http://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>`_.
Before continuing, read the "Background" section of that tutorial, and brush up
on any of the readings from section 4 that you need to. Ignore
"Prerequisites"--our workspace is already set up for you, and we'll walk through
instructions for building your code here.

This section is by far the most difficult of the tutorial. If you've made it
this far, though, you should have everything you need for this section *except
for* C++ knowledge. This is a real hurdle. If you already have Java or C
experience, the syntax is similar enough to where you'll be able to work
through this section, even if it takes you some time. If you aren't so lucky,
read through the sections "Basics of C++", "Program structure", and "Classes"
of `the C++ tutorial <https://cplusplus.com/doc/tutorial/>`_ and try your best.

**Read the rest of this section before starting.**

Objective
~~~~~~~~~

In this section, you'll be creating a SoccerMom node that gets the team color
and picks a fruit to match. Our robots have to stay motivated somehow!

You can find the team color by subscribing to the relevant topic (this should
become obvious after looking at the list of topics). To "pick a fruit", publish
a `standard string msg
<http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html>`_ to a new
topic `/team_fruit`.
 * When our team color is yellow, publish "banana" to `/team_fruit`.
 * When our team color is blue, publish "blueberries" to `/team_fruit`.

Creating a New Node
~~~~~~~~~~~~~~~~~~~

Often in C++ you'll see the use of a header file, which ends in `.hpp`, and a
source file, which ends in `.cpp`. Header files contain all the function
declarations and docstrings explaining their use. Source files contain the
function definitions--that is, the code that actually makes the functions work.
This allows for many files to share access to the same methods or classes
without copy-pasting their entire implementation by importing the right header
files. 

(For more information, check out `this
<https://cplusplus.com/articles/Gw6AC542/>`_ resource.)

Let's take a look at a real example in our codebase to make this more
understandable. Find the radio.cpp and radio.hpp files in our codebase. In the
last section, you used ``rqt`` to launch the Node Graph. One of the nodes that
subscribe and publish to various topics is ``/radio``, and these files are the
source of that node. 

Comparing the similarities and differences between the subscribers and
publishers in these files vs. the ROS tutorial will help you learn what you can
take directly from the ROS tutorial, and where you need to deviate from it.

As a brief overview to help you get started...

* Notice the ``#includes`` at the top of both files. ``#includes`` are like
  ``import`` statements from Java or Python (with slight differences that are
  not terribly important for our purposes right now). Using ROS forces you to
  include certain things; again, check out the ROS tutorial.

* The header file defines Radio to be subclass of rclcpp::Node (see `: public
  rclcpp::Node``). This means the Radio has access to all the
  methods of rclcpp::Node (notice that Node is under namespace rclcpp!).

* The header file also categorizes all variables and methods of the Radio
  class into ``public``, ``protected``, and ``private``. These are known
  as "access specifiers". `This
  article <https://www.w3schools.com/cpp/cpp_access_specifiers.asp#:~:text=In%20C%2B%2B%2C%20there%20are,be%20accessed%20in%20inherited%20classes.>`_
  sums them up nicely.

* Both files are enclosed under a namespace. Namespaces are an organizational
  tool in C++ which helps organize large codebases. For instance, the radio.hpp
  file defines ``namespace radio``, so when other files use the ``SimRadio``
  object, they reference ``radio::SimRadio``. Give your SoccerMom node a
  ``tutorial`` namespace.

* The existing codebase makes heavy use of *lambda expressions*. For instance,
  in radio.cpp:

.. code-block::

   create_subscription<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_topic(i), rclcpp::QoS(1), [this,
            i](rj_msgs::msg::ManipulatorSetpoint::SharedPtr manipulator) {  //
            NOLINT
                manipulators_cached_.at(i) = *manipulator;
            });

Here, a lambda expression is used instead of the callback function that you'll
see in the ROS tutorial. A lambda expression is just a concise way of defining
a function without giving it a name. This is only suitable when you know you
don't want to reuse a function (since without a name, you can't reference that
function anywhere else). and requires less lines of code when compared to
having another function. 

Read more `here <https://www.programiz.com/cpp-programming/lambda-expression>`_
if you would like.

 * The existing codebase also makes heavy use of *pointers*. You will see this
   in the use of the arrow operator, ``->``. For example:

.. code-block::

   robot_status_topics_.at(robot_id)->publish(robot_status);

The arrow operator is used to access a method or element of an object, when
given a pointer to that object. Above, ``robot_status_topics_`` is a list of
pointers to ROS publisher objects. Calling ``->publish(robot_status)`` on one
element in that list publishes a robot status using that specific publisher.
You will learn more about pointers when you take CS 2110, but if you want to
get a headstart, see `this
resource <https://www.tutorialspoint.com/cplusplus/cpp_member_operators.htm>`_.

* Finally, the docstrings in the radio header file state that the Radio class
  abstract superclass of the network_radio and sim_radio nodes. (If you are
  unfamiliar with the concept of abstraction, `here
  <https://www.pythontutorial.net/python-oop/python-abstract-class/>`_ is more
  information.) The concrete subclasses are NetworkRadio and SimRadio.

You might be wondering: okay, this is great, but how do I compile and run my
new node?

Well, both NetworkRadio and SimRadio have an associated <name>_main.cpp file
(e.g. ``sim_radio_node_main``) which contains the main function for its
respective node. This structure is intended to make writing the CMake files for
the directory easier. We use `CMake <https://cmake.org/overview/>`_ to compile
our C++ programs on a variety of different hardware architectures. 

As a result, to compile and use your new node, you'll need to add your new
source files to the right CMake files.

Building Your Node
~~~~~~~~~~~~~~~~~~

CMakeLists.txt files are used to make standard build files for the directory. It
locates files, libraries, and executables to support complex directory
hierarchies. Locate the CMakeLists.txt file in
``robocup-software/soccer/src/soccer``.

Let's start looking at all the magic CMake text that builds our cpp code:

* Notice the source files under ``ROBOCUP_LIB_SRC``. You will find the
  radio files that you explored earlier, along with all the other source
  files we use (motion control, UI, etc.).

* Many of the nodes have an environment variable set for their
  <node>_main.cpp. For instance, SimRadio has the line
  ``set(SIM_RADIO_NODE_SRC radio/sim_radio_node_main.cpp)``. This defines
  ``SIM_RADIO_NODE_SRC`` to be the filepath
  ``radio/sim_radio_node_main.cpp``. You will need a similar line for
  your new node, with adjustments to the names.

* There is a corresponding ``target_sources`` line that SimRadio needs to
  actually start: ``target_sources(sim_radio_node PRIVATE
  ${SIM_RADIO_NODE_SRC})``

The rest is up to you. Keep using SimRadio as an example. Search through and
find the parts of the CMake file where SimRadio is used, then follow that
format for your own node. 

It's okay if you don't understand everything that's going on. (Honestly, CMake
files are one of those things we re-learn when adding new nodes and forget
almost immediately after.) Just match the existing patterns.


Launching Your Node
~~~~~~~~~~~~~~~~~~~

You're almost there! The final file to get your node up and running is the
``.launch`` file.

Launch files in ROS are a convenient way of starting up multiple nodes, setting
initial parameters, and other requirements. Find the ``robocup-software/launch``
directory and open the file that seems most relevant to your new node.
(HINT: Your node should be located in ``robocup-software/soccer``.) 

Like the CMake section, this part is a lot of copying what already exists and
changing it to match your new node's names. If you want to read more about ROS
launch files, `the tutorial
page<https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html>`_
is a great place to start.


Testing
~~~~~~~

Whew! What a section. If you've made it this far, you should have everything
you need to create the SoccerMom node. 

This section will probably take you a while. Remember, when you run into
issues, your order of question-asking should be:

#. Google

#. FAQ page in our docs

#. Fellow new members

#. Software lead

#. Anyone the SW lead takes advice from

.. note::

   Since you have made changes to the C++ part of our codebase, you must build
   it again to test your node. This may take a while, so be patient and
   proactive with your changes. If you forgot how to build the codebase, go to
   the Getting Started page.

To test, change our team color using the UI by going to the top menu bar and
clicking Field > Team Color. You should see the team color change in the top
right corner of our UI. Screenshot proof that your `/team_fruit` topic is
publishing the right fruit for both options, and post as a comment to your PR.

Similar to the Python section, there's a lot of file-finding in this part. Use
the option in your IDE or text editor that allows you to see a full folder at
once. For instance, in VS Code, there is an option to open a full folder, which
displays all the subfolders and files in the left toolbar.

If you've read this whole section and are feeling a little intimidated, that's
normal. The paragraphs above form a nice guide and checklist for you to follow.
Just try your best, one step at a time, and eventually you'll have a working
piece of software to be proud of.

7. Conclusion
-------------

Finally, tag your software lead for review on your pull request. For your final
comment, leave feedback on anything that confused you in this tutorial. When
reviewing your PR, your software lead will either request changes, meaning they
have some feedback for you to adjust your PR, or approve it, meaning your
changes are ready to merge.

However, this time, upon approval, **CLOSE your pull request. Do not merge it.**
Since this is only a tutorial project, there's no need to add it to the
codebase.

Congratulations! This was a long journey, but if you've made it this far, you
have proved yourself worthy of your teammates' trust, and are ready to work on
real features. We hope this was a helpful first step in your long robotics
career.
