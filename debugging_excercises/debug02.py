#%%
import numpy as np


#%% 
a = np.array([1, 2, 3])
b = np.array([[1], [2]])
result = a + b

#TODO answer the following questions as comments below:
# What is the shape of a?
# What is the shape of b?
# What is the shape of result?
# Why did this happen? and what occurred? 



#%% 
a = np.array([1, 2, 3])
b = np.array([4])
result = a * b 

#TODO answer the following questions as comments below:
# Is the output what you expected? Can you explain how it happened?


#%% 
a = np.ones((2, 3))
b = np.ones((3, 2))
result = a + b  

# TODO answer the following questions as comments below:
# Why doesn't this work? If the 2D arrays are meant to be the same size, fix 
# the code so that you can successfully add them. 


#%% 

a = np.array([1, 2, 3])
b = np.array([[1], [2], [3]])
result = a + b

#TODO - check the output of this addition, is it what you expected? Answer the 
# following questions as comments below:
#  
#  What is the shape of a and the shape of b? 
#  Why is the result 3x3? and what does the addition do in this case?



#%%
a = np.array([1, 2, 3])
b = np.array(np.eye(3))
result1 = b*a
result2 = np.dot(b,a)
result3 = b@a

# which of these is the intended result? 
# TODO put your answer here as a comment and justify it. 


#%%
a_column = a.reshape((3,1)) 
result1 = b*a_column
result2 = np.dot(b,a_column)
result3 = b@a_column 

# which of these is the intended result? 
# TODO put your answer here as a comment and justify it. 


#%%
result1 = a_column.T*a
result2 = a_column*a 
result3 = a*a_column
result4 = np.dot(a_column,a)
result5 = np.dot(a_column,a_column)
result6 = np.dot(a_column.T, a_column)
result7 = a_column.T@a_column

#TODO: answer these questions as comments below: 
#  Why does np.dot(a, a) work? but np.dot(a_column, a_column) doesn't? 
#  If I'm trying to do find the sum of the squares of the elements of a, which of these should I use?





# if you still have questions about broadcasting, and array dimensions, please see the following:
# https://numpy.org/doc/stable/user/basics.broadcasting.html 
# https://www.youtube.com/watch?v=oG1t3qlzq14 