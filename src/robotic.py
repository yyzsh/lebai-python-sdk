import grpc
import robot_controller_pb2
import robot_controller_pb2_grpc
import logging

def run():
    # NOTE(gRPC Python Team): .close() is possible on a channel and should be
    # used in circumstances in which the with statement does not fit the needs
    # of the code.
    with grpc.insecure_channel('192.168.3.218:5181') as channel:
        stub = robot_controller_pb2_grpc.RobotControllerStub(channel)
        response = stub.GetJointTemp(robot_controller_pb2.IntRequest(index=0))
    print(f"Greeter client received: {response.degree:.3f}")

if __name__ == '__main__':
    logging.basicConfig()
    run()
