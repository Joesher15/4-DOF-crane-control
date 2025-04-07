from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

left_arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="shoulder",
      origin_translation=[-10, 0, 5],
      origin_orientation=[0, 1.57, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow",
      origin_translation=[25, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="wrist",
      origin_translation=[22, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
])

print(left_arm_chain.inverse_kinematics([2, 2, 2]))
