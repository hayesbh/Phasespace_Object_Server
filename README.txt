The core_object_server currently is somewhat slow
  It will be able to be sped up by:
    1. Connecting it to a built in Point/Vector and Quaternion library
      that exploits the gpu - Specifically for finding rotation
    2. Passing vectors by reference to avoid copying excessive amount of data
    3. Having a universal Point set  that is updated once
         And each object just contains pointers to those Points
	 This I think will speed it up the most
	 Currently there is a lot of passing around and searching through the updated point set. I am curious as to whether we even need a Point class and just use their Marker's instead. But this could lead to potential issues in the future if we decide to change the API.
    4. My guess is that it is the Object updating that is taking up the most processor time.  The Points by reference will help this. Make sure that every function for updating is only called once per Update() call.

  Things that most likely maxed out
    1. Object Collision Detection (only could be sped up by math library)
  Things that take up most time probably:
    1. Updating points in all the objects
    2. Going through all the calculations for rotation
  Things to test:
    1. Try out a longer use case scenario to see if it is robust
    2. Try with more objects in the scene and for longer
