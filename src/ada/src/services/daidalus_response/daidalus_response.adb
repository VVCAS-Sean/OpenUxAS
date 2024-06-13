with Ada.Containers;             use Ada.Containers;
with AVTAS.LMCP.Types;           use AVTAS.LMCP.Types;
with UxAS.Comms.LMCP_Net_Client; use UxAS.Comms.LMCP_Net_Client;
with LMCP_Messages;              use LMCP_Messages;
with Ada.Text_IO;                use Ada.Text_IO;
with Common;                     use Common;

-- __TODO__
-- Include any other necessary packages.

package body Daidalus_Response with SPARK_Mode is

   -- __TODO__
   -- Include any local types or use clauses you would like to have.
   --
   -- __Example__
   --
   -- use all type Pos64_Nat64_Maps.Formal_Model.M.Map;
   -- use Pos64_Vectors.Formal_Model.M;

   -- __TODO__
   -- Declare and define bodies for any local subprograms used internally in the
   -- body of the package. This may include helper subprograms or ghost code
   -- (e.g. lemmas) to help with proof.
   -- 
   -- __Example__
   -- 
   -- procedure Lemma_Mod_Incr (A : Natural; B : Positive) with
   --   Ghost,
   --   Pre => A < Integer'Last,
   --   Post =>
   --     (if A mod B = B - 1 then (A + 1) mod B = 0
   --        else (A + 1) mod B = A mod B + 1);
   -- 
   -- procedure Lemma_Mod_Incr (A : Natural; B : Positive) is null;
   --
   -- --------------------
   -- -- Construct_Path --
   -- --------------------
   -- 
   -- procedure Construct_Path (...) with
   --   Pre => ...
   --   Post => ...;
   -- 
   -- procedure Construct_Path (...)
   -- is
   --   ...
   -- begin
   --   ...
   -- end Construct_Path;

   -- __TODO__
   -- Define bodies for any subprograms declared in the package specification.
   -- These are likely to include procedures to handle SPARK-compatible LMCP
   -- messages (by convention named `Handle_<MessageType>`), along with other
   -- SPARK subprograms needed by the service. Note that procedures that send
   -- SPARK-compatible LMCP messages directly should include the service's
   -- mailbox as a parameter. Also, as a general tip for proof, subprograms that
   -- have complex contracts and operate on the state should in their
   -- implementations rely on helper subprograms that operate over *only* the
   -- required fields of the state and have contracts that can be leveraged for
   -- proof of the original subprogram's contract. This modularizes proof and
   -- minimizes context for the provers, making proof more tractable.
   -- 
   -- __Example Stubs__
   --
   -- ---------------------------
   -- -- Handle_MissionCommand --
   -- ---------------------------
   --
   -- procedure Handle_MissionCommand
   --   (State : in out <Service_Name>_State;
   --    MC : MissionCommand)
   -- is
   --    ...
   -- begin
   --    ...
   -- end Handle_MissionCommand;
   --
   -- ---------------------
   -- -- Produce_Segment --
   -- ---------------------
   -- 
   -- procedure Produce_Segment
   --   (State : in out <Service_Name>_State;
   --    Config : <Service_Name>_Configuration_Data;
   --    Mailbox : in out <Service_Name>_Mailbox)
   -- is
   --   ...
   -- begin
   --   ...
   --   Construct_Path (...);
   --   ... 
   -- end Produce_Segment;
   procedure Process_WellclearViolation_Message 
     (m_DAIDALUSResponseServiceState : in out Daidalus_Response_State;
      m_DAIDALUSResponseServiceConfig : Daidalus_Response_Configuration_Data;
      WCV_Intervals : LMCP_Messages.WellClearViolationIntervals) is
   begin
      if Common.Int64 (WCV_Intervals.EntityID) = m_DAIDALUSResponseServiceConfig.
        VehicleID
      then
         null;
      end if;
      
   end Process_WellclearViolation_Message;
   
   procedure Process_DAIDALUSConfiguration_Message 
     (m_DAIDALUSResponseServiceState : in out Daidalus_Response_State;
      m_DAIDALUSResponseServiceConfig : Daidalus_Response_Configuration_Data;
      ConfigurationMessage : LMCP_Messages.DAIDALUSConfiguration)
   is 
   begin
      --Set State parameters from DAIDALUSConfiguration message when message is 
      --for configured ownship -------------------------------------------------
      if Common.Int64 (ConfigurationMessage.EntityID) = 
        m_DAIDALUSResponseServiceConfig.VehicleID 
      then
         m_DAIDALUSResponseServiceState.ReadyToAct := True;
         m_DAIDALUSResponseServiceState.Altitude_Min_m := ConfigurationMessage.
           MinAltitude;
         m_DAIDALUSResponseServiceState.Altitude_Max_m := ConfigurationMessage.
           MaxAltitude;
         m_DAIDALUSResponseServiceState.Altitude_Interval_Buffer_m := 
           ConfigurationMessage.AltitudeStep / 2.0;
         m_DAIDALUSResponseServiceState.Heading_Interval_Buffer_deg := 
           ConfigurationMessage.TrackStep / 2.0;
         m_DAIDALUSResponseServiceState.GroundSpeed_Interval_Buffer_mps := 
           ConfigurationMessage.GroundSpeedStep / 2.0;
         m_DAIDALUSResponseServiceState.GroundSpeed_Min_mps :=
           ConfigurationMessage.MinGroundSpeed;
         m_DAIDALUSResponseServiceState.GroundSpeed_Max_mps := 
           ConfigurationMessage.MaxGroundSpeed;
         
      end if;

   end Process_DAIDALUSConfiguration_Message;

end Daidalus_Response;
