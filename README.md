## The Goal of this Project

In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car 
to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, 
avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map 
data.

Included in the submission should be a writeup, either in the README or a seperate file that details how the 
project was completed.

A great writeup should include the rubric points as well as your description of how you addressed each point. 
You should include a detailed description of the code used in each step (with line-number references and code 
snippets where appropriate) and links to other supporting documents or external references.

All that said, please be concise! We're not looking for you to write a book, just a brief description of how 
you passed each rubric point, and references to the relevant code :)

### Lesson 2:
This lesson is taught by Sebastian Thrun (Udacity's former CEO) and comes from one of Udacity's first courses. 
The production style is a little different than what you may be used to but the content is very good.

This lesson covers discrete path planning and in the last lesson you will learn about continuous path 
planning. Even though the real world is continuous, there are many situations where discretizing the world 
makes it easier and computationally faster to solve path planning problems.

In addition to the practical benefits of these algorithms, it's also conceptually useful to learn about them 
because they deal with some of the same concepts that we will keep coming back to in this lesson. Those 
concepts include:

* Cost functions and how to include human insights (like "it's easier to make right turns than left turns") 
  into our planning algorithms.

* Optimality and the tradeoffs associated with finding the best solution vs finding a solution that is good 
  enough.

* Online vs Offline algorithms and how we can avoid complex computation on the road by precomputing best paths 
  whenever possible.
