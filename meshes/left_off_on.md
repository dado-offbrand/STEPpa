LEFT OFF ON:
The leg positions are wrong not due to incorrect math (however, the leg
starting angles being negative may be an issue in the future), but likely due to
the mismapping of order of joints in the math vs sim. There needs to be a set
dictionary that has the leg group order, with their index in sim.


1
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
0.0

2
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
0.0

3
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
0.0

4
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-45.0

5
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-45.0

6
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-45.0

7
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-45.0

8
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-90.0

9
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-90.0

10
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-90.0

11
shoulders: 0, 3, 6, 9. Should be 0deg
Upper legs: 1, 4, 7, 10. Should be -45deg
Lower legs: 2, 5, 8, 11. Should be -90deg
-90.0

        # for i in range(len(data.ctrl)):
        #     print(i)
        #     print("shoulders: 0, 3, 6, 9. Should be 0deg")
        #     print("Upper legs: 1, 4, 7, 10. Should be -45deg")
        #     print("Lower legs: 2, 5, 8, 11. Should be -90deg")
        #     print(np.rad2deg(positions[i]))
        #     input()
        #     data.ctrl[i] = positions[i]
        # print("\n\n\n")

quadruped gait isn't gaiting