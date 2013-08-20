The core_object_server currently is somewhat slow
  It will be able to be sped up by:
    1. Connecting it to a built in Point/Vector and Quaternion library
      that exploits the gpu
    2. Passing vectors by reference to avoid copying excessive amount of data
    3. Having a universal Point set that is updated once
         And each object just contains pointers to those Points
    4. My guess is that it is the Object updating that is taking up the most processor time.  The Points by reference will help this. Make sure that every function for updating is only called once per Update() call.

  Things that most likely maxed out
    1. Object Collision Detection (only could be sped up by math library)
