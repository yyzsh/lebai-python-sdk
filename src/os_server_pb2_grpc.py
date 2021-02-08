# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import os_server_pb2 as os__server__pb2


class OsServerStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.getRingData = channel.unary_stream(
                '/osserver.OsServer/getRingData',
                request_serializer=os__server__pb2.Req.SerializeToString,
                response_deserializer=os__server__pb2.RingData.FromString,
                )


class OsServerServicer(object):
    """Missing associated documentation comment in .proto file."""

    def getRingData(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_OsServerServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'getRingData': grpc.unary_stream_rpc_method_handler(
                    servicer.getRingData,
                    request_deserializer=os__server__pb2.Req.FromString,
                    response_serializer=os__server__pb2.RingData.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'osserver.OsServer', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class OsServer(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def getRingData(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/osserver.OsServer/getRingData',
            os__server__pb2.Req.SerializeToString,
            os__server__pb2.RingData.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
