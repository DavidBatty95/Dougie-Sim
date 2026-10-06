"""Single-producer command epochs and freshness for the planner/supervisor hop.

Identity prevents accidental replay across restarts; it is not authentication or
the upstream navigation/teleoperation ownership arbiter.
"""
import math


def goal_signature(goal):
    p,q = goal.pose.position,goal.pose.orientation
    return [goal.header.frame_id,goal.header.stamp.sec,goal.header.stamp.nanosec,
            p.x,p.y,p.z,q.x,q.y,q.z,q.w]


class CommandEnvelopeGate:
    def __init__(self):
        self.publisher = None
        self.sequence = 0
        self.stamp = -math.inf
        self.valid_after = -math.inf
        self.retired_publishers = set()
        self.rejections = {}

    def fence(self,stamp):
        self.valid_after = max(self.valid_after,stamp)

    def reject(self,reason):
        self.rejections[reason] = self.rejections.get(reason,0)+1
        return reason

    def accept(self,data,now,frame,goal,active,max_age):
        if not isinstance(data,dict):
            return self.reject('invalid_shape')
        publisher,sequence,stamp = data.get('publisher_id'),data.get('sequence_id'),data.get('stamp')
        if (type(data.get('version')) is not int or data['version'] != 1 or not isinstance(publisher,str) or not 0<len(publisher)<=128 or
                type(sequence) is not int or sequence<=0 or type(stamp) not in (int,float) or not math.isfinite(stamp)):
            return self.reject('invalid_identity')
        if data.get('frame') != frame or type(data.get('active')) is not bool or data['active'] != active or data.get('goal') != goal:
            return self.reject('intent_mismatch')
        if not 0.<=now-stamp<=max_age or stamp<self.valid_after:
            return self.reject('expired_or_prefence')
        vectors = [data.get('first')]
        if data.get('then') is not None:
            vectors.append(data['then'])
        if any(not isinstance(v,list) or len(v)!=3 or
               any(type(x) not in (int,float) or not math.isfinite(x) for x in v) for v in vectors):
            return self.reject('invalid_command')
        duration = data.get('duration')
        if (type(duration) not in (int,float) or not math.isfinite(duration) or not 0.<=duration<=10. or
                not isinstance(data.get('state'),str)):
            return self.reject('invalid_schedule')
        if publisher in self.retired_publishers:
            return self.reject('retired_publisher')
        if publisher == self.publisher:
            if sequence<=self.sequence or stamp<self.stamp:
                return self.reject('out_of_order')
        elif self.publisher is not None:
            if now-self.stamp<=max_age:
                return self.reject('publisher_busy')
            self.retired_publishers.add(self.publisher)
        self.publisher,self.sequence,self.stamp = publisher,sequence,stamp
        return None
