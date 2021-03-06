with UxAS.Comms.LMCP_Net_Client.Service.Example_Spark_Service; use UxAS.Comms.LMCP_Net_Client.Service.Example_Spark_Service;

with afrl.cmasi.lmcpTask.SPARK_Boundary; use afrl.cmasi.lmcpTask.SPARK_Boundary;
with afrl.cmasi.MissionCommand; use afrl.cmasi.MissionCommand;
with afrl.cmasi.AutomationResponse; use afrl.cmasi.AutomationResponse;
with afrl.cmasi.MissionCommand.SPARK_Boundary; use afrl.cmasi.MissionCommand.SPARK_Boundary;
with afrl.cmasi.AutomationResponse.SPARK_Boundary; use afrl.cmasi.AutomationResponse.SPARK_Boundary;

with afrl.cmasi.AutomationRequest; use afrl.cmasi.AutomationRequest;
with afrl.cmasi.AutomationRequest.SPARK_Boundary; use afrl.cmasi.AutomationRequest.SPARK_Boundary;
with avtas.lmcp.object.SPARK_Boundary; use avtas.lmcp.object.SPARK_Boundary;
with Ada.Containers; use Ada.Containers;

private
package UxAS.Comms.LMCP_Net_Client.Service.Example_Spark_Service.SPARK with SPARK_Mode is
   use all type Int64_Set;

   --------------------------------------
   -- Functions for annotation purpose --
   --------------------------------------

   function Recognized_VehicleId_From_Previous_AutomationResponse
     (This : Example_Spark_Service; Id : Int64) return Boolean is
     (Int64_Sets.Contains(This.Configs.AutomationIds, Id));

   -------------------------------------
   -- Regular Service Functionalities --
   -------------------------------------

   procedure Handle_MissionCommand
     (This          : in Example_Spark_Service;
      Command       : My_Object_Any;
      Recognized_Id : out Boolean)
   with
     Pre => Deref(Command) in MissionCommand,
     Post =>
       (if Recognized_VehicleId_From_Previous_AutomationResponse
             (This, MissionCommand(Deref(Command)).getVehicleID)
        then Recognized_Id else not Recognized_Id);

end UxAS.Comms.LMCP_Net_Client.Service.Example_Spark_Service.SPARK;
