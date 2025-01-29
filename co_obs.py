from examples.discrete_belief.dist import DDist
from belief import BeliefState
import numpy as np
import json

class CorrelationalObsModel():
    def __init__(self, use_correlation: bool, alpha = 2.0):
        self.alpha = alpha
        self.use_correlation = use_correlation
        if self.use_correlation:
            self.sim_map = {("apple", "banana"): 0.36482}

    def amplify(self, sim):
        mult_val = sim * self.alpha
        if mult_val >= 0.9: mult_val = 0.9
        elif mult_val <= -0.9: mult_val = -0.9
        return mult_val
    
    def get_observation_fn(self, loc, p_look_fp, p_look_fn, visibility):
        """
        loc         : the true location of the object
        p_look_fp   : probability of false positive detection
        p_look_fn   : probability of false negative detection
        visibility  : factor in [0, 1], how visible the object is
        """
        # Clamp visibility to [0, 1] just in case
        visibility = max(0.0, min(1.0, visibility))

        def observation_fn(query_loc):
            """
            Returns a distribution over {True, False} for "object detected"
            given that we are 'looking' at query_loc.
            """
            if query_loc == loc:
                # Object is actually here
                # Base detection prob is (1 - p_look_fn),
                # scale by visibility
                p_true = (1 - p_look_fn) * visibility
            else:
                # Object is actually NOT here
                # Base false-positive prob is p_look_fp,
                # scale by visibility
                p_true = p_look_fp * visibility

            # Probability of "detected = False"
            p_false = 1 - p_true

            return DDist({True: p_true, False: p_false})

        return observation_fn
    
    def get_pos_correlation_fn(self, loc, num_loc, sim):
        def fn(l):
            if l == loc:
                P_false, P_true = 1, 0
            else:
                P_false, P_true = 0, 1
            P_true = sim * P_true + (1 - sim) * (1/num_loc)
            P_false = sim * P_false + (1 - sim) * (1/num_loc)
            return DDist({True: P_true,
                        False: P_false})
        return fn
    
    def get_neg_correlation_fn(self, loc, num_loc, sim):
        def fn(l):
            if l == loc:
                P_false, P_true = 0, 1
            else:
                P_false, P_true = 1, 0
            P_true = abs(sim) * P_true + (1 + sim) * (1/num_loc)
            P_false = abs(sim) * P_false + (1 + sim) * (1/num_loc)
            return DDist({True: P_true,
                        False: P_false})
        return fn
    
    def correlationUpdate(self, target_item:str, detected_items: set, target_loc: str, surf_visibility:float, 
                        p_fp: float, p_fn: float, state: BeliefState, obs: bool):
        # Create observation model
        num_surf = len(state.surfaces)
        surf_obs_model = self.get_observation_fn(target_loc, p_fp, p_fn, surf_visibility)
        # Update detected object belief
        state.belief[target_item].obsUpdate(surf_obs_model, obs)
        # Update other object belief based on correlation
        if self.use_correlation and not obs:
            print("Correlation update", detected_items)
            for obj in detected_items:
                # if obj was not already detected
                if obj not in state.known and obj != target_item:
                    sim = self.sim_map[(target_item, obj)]
                    # Create correlation model
                    if sim >= 0:
                        print("Positive correlation: ", target_item, obj, sim)
                        surf_corr_model = self.get_pos_correlation_fn(target_loc, num_surf, sim)
                    else:
                        print("Negative correlation: ", target_item, obj, sim)
                        surf_corr_model = self.get_neg_correlation_fn(target_loc, num_surf, sim)
                    state.belief[target_item].obsUpdate(surf_corr_model, obs)