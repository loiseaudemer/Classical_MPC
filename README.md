# Classical_MPC
matlab code for two papers I read recently:
Paper 1: 
        Title: A quasi-infinite horizon nonlinear model predictive control scheme with guaranteed stability
        Authors: H. Chen and F. Allgower
        Year: 1998, Automatica

        Contributions: 
        can be found in the paper
        
        Drawbacks:
                 (i) This method is only applicable to a limited range of nonlinear systems. The systems to be controlled must
                     stabilizable at the origin after linearization.
                 (ii) Not applicable to non-holomorphic systems.
        
        See paper 2 for control methods of non-holomorphic systems.
                 
Paper 2:
        Title: A general framework to design stabilizing nonlinear model predictive control
        Author(s): F. Fontes
        Year: 2001, Systems & control letters
        
        Contributions: 
        Propose a general framework for nmpc, which consists of Chen's schemes. The range of nonlinear systems
        can be controlled are extended, including nonholomorphic systems.
        
        Drawbacks: 
        No explicit method of finding mpc control parameters (N, prediction horizon; T, sampling time, P terminal penalty
        matrix; stage cost function, etc..) is provided. Sometimes it might be difficult to find the feasible parameters.
        
Notes:
        1. MPC algorithms proposed in the two papers are verified.
        2. You can test your system dynamics and cost functions with the Matlab code provided in this repository.
        3. You can plot state trajectories, system input profiles and cost funcition values as well.
        4. A more general version of .m code for nmpc will be released later.
