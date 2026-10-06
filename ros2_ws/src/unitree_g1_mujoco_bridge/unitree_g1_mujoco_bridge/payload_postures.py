"""Versioned right-arm postures used by the cycle-01 payload training fixture.

Joint order is shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw.
These are simulation training postures, not calibrated physical grasp poses.
"""
PAYLOAD_V1_POSTURES = (
    (-.92,-1.04,.01,-.20,1.45,.89,-.65),
    (-1.10,-.60,.10,-.10,1.10,.94,-.70),
    (-.54,-.76,.05,-.27,.94,.62,-.43),
)
