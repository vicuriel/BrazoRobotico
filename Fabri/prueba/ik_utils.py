import math
from typing import List, Tuple, Optional

# choose_solution: select best IK solution from candidates
def choose_solution(sols: List[Tuple[float,float,float,float]], q_prev: List[float], limits: Optional[dict]=None) -> Optional[Tuple[float,float,float,float]]:
    """Select the best solution among IK candidates.

    Heuristics used (in order of importance):
      - Discard solutions out of `limits` if provided
      - Minimal total joint change from `q_prev` (L1 norm)
      - Penalize solutions close to joint limits

    Args:
      sols: list of candidate tuples (q1,q2,q3,q4)
      q_prev: previous joint vector [q1,q2,q3,q4]
      limits: optional dict {'q1':(min,max), ...}

    Returns:
      best solution tuple or None if none valid
    """
    if not sols:
        return None

    def in_limits(q):
        if not limits:
            return True
        keys = ['q1','q2','q3','q4']
        for val, k in zip(q, keys):
            if k in limits:
                mn, mx = limits[k]
                if val < mn - 1e-9 or val > mx + 1e-9:
                    return False
        return True

    candidates = [tuple(map(float, s)) for s in sols if in_limits(s)]
    if not candidates:
        return None

    best = None
    best_score = float('inf')

    for q in candidates:
        # basic proximity cost to previous pose
        change_cost = sum(abs(qi - pi) for qi, pi in zip(q, q_prev))

        # penalty near limits
        penalty = 0.0
        if limits:
            keys = ['q1','q2','q3','q4']
            for qi, k in zip(q, keys):
                if k in limits:
                    mn, mx = limits[k]
                    span = mx - mn
                    if span > 0:
                        dist = min(qi - mn, mx - qi)
                        threshold = 0.1 * span
                        if dist < threshold:
                            penalty += (threshold - dist) / threshold

        # final score: change_cost with higher weight plus penalty
        score = change_cost + 10.0 * penalty
        if score < best_score:
            best_score = score
            best = q

    return best
