import struct

from unitree_hg.msg import LowCmd

from unitree_g1_mujoco_bridge.unitree_crc import lowcmd_crc,lowcmd_payload


def test_hg_crc_matches_vendor_bitwise_reference_and_layout():
    command = LowCmd(mode_pr=0,mode_machine=5)
    for index,motor in enumerate(command.motor_cmd):
        motor.mode = 1 if index<29 else 0
        motor.q,motor.dq,motor.tau = index*.013,-index*.02,index*.1
        motor.kp,motor.kd = 60.,2.5
    payload = lowcmd_payload(command)
    assert len(payload)==1000
    assert payload[:4]==b'\x00\x05\x00\x00'
    assert struct.unpack_from('<f',payload,8)[0]==command.motor_cmd[0].q
    reference = 0xffffffff
    for (word,) in struct.iter_unpack('<I',payload):
        for bit in range(31,-1,-1):
            high = reference & 0x80000000
            reference = (reference << 1) & 0xffffffff
            if high:
                reference ^= 0x04C11DB7
            if word & (1 << bit):
                reference ^= 0x04C11DB7
    assert lowcmd_crc(command)==reference
    command.motor_cmd[28].q += .01
    assert lowcmd_crc(command)!=reference
    command.mode_machine = 4
    assert lowcmd_crc(command)!=reference
