with Ada.Containers;             use Ada.Containers;
with AVTAS.LMCP.Types;           use AVTAS.LMCP.Types;
with UxAS.Comms.LMCP_Net_Client; use UxAS.Comms.LMCP_Net_Client;
with LMCP_Messages;              use LMCP_Messages;
with Ada.Text_IO;                use Ada.Text_IO;
with Common;                     use Common;
with definitions;      
with SPARK.Containers.Functional.Vectors;

-- __TODO__
-- Include any other necessary packages.

package body Daidalus_Response with SPARK_Mode is
   
   -- Helper functions ---------------------------------------------------------
   procedure CreateAltitudeBands (LMCP_Altitudes : AltitudeInterval; 
                                  LMCP_AltitudeZone : BandsRegion_seq;
                                  DAIDALUS_Altitude_Bands : out 
                                    definitions.OrderedIntervalVector) with 
       Post => definitions.Are_Legitimate_Bands (DAIDALUS_Altitude_Bands);
   
   procedure CreateAltitudeBands (LMCP_Altitudes : AltitudeInterval;
                                  LMCP_AltitudeZone : BandsRegion_seq;
                                  DAIDALUS_Altitude_Bands : out 
                                 definitions.OrderedIntervalVector) is
      result : definitions.OrderedIntervalVector;
   begin
      pragma Assume (Generic_Real64_Sequences.Iter_First 
                     (LMCP_Altitudes.Altitude) = 
                       BandsRegion_sequences.Iter_First 
                         (LMCP_AltitudeZone) 
                     and then Generic_Real64_Sequences.Last 
                       (LMCP_Altitudes.Altitude) = 
                         BandsRegion_sequences.Last 
                         (LMCP_AltitudeZone));
      if Generic_Real64_Sequences.Iter_First (LMCP_Altitudes.Altitude) = 
            BandsRegion_sequences.Iter_First (LMCP_AltitudeZone) and then 
          Generic_Real64_Sequences.Last (LMCP_Altitudes.Altitude) =
            BandsRegion_sequences.Last (LMCP_AltitudeZone)
      then
         for Index in BandsRegion_sequences.Iter_First 
           (LMCP_AltitudeZone) .. 
             BandsRegion_sequences.Last (LMCP_AltitudeZone) loop
            declare
               temp_interval : definitions.interval;
            begin
               temp_interval.LowerBound := Generic_Real64_Sequences.Get 
                 (LMCP_Altitudes.Altitude, Index)(1);
               temp_interval.UpperBound := Generic_Real64_Sequences.Get 
                 (LMCP_Altitudes.Altitude, Index)(2);
               case BandsRegion_sequences.Get (LMCP_AltitudeZone, Index) is
                  when LMCP_Messages.MID => temp_interval.Classification := 
                       definitions.Mid;
                  when LMCP_Messages.NEAR => temp_interval.Classification :=
                       definitions.Near;
                  when LMCP_Messages.FAR => temp_interval.Classification :=
                       definitions.Far;
               end case;
               MyVectorOfIntervals.Append (result, temp_interval);
            end;
            DAIDALUS_Altitude_Bands := result;
            pragma Assume (Are_Legitimate_Bands (DAIDALUS_Altitude_Bands));
         end loop;
      else
         null;
      end if;      
             
   end CreateAltitudeBands;

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
      WCVdata : WCV_data;
   begin
      if Common.Int64 (WCV_Intervals.EntityID) = m_DAIDALUSResponseServiceConfig.
        VehicleID
      then
         --Configure WCVdata object with paramater for automatic response-------
         WCVdata.CurrentState.altitude_m := WCV_Intervals.CurrentAltitude;
         WCVdata.CurrentState.groundSpeed_mps := 
           WCV_Intervals.CurrentGroundSpeed;
         WCVdata.CurrentState.heading_deg := WCV_Intervals.CurrentHeading;
         WCVdata.CurrentState.verticalSpeed_mps := WCV_Intervals.
           CurrentVerticalSpeed;
         WCVdata.CurrentState.latitude_deg := WCV_Intervals.CurrentLatitude;
         WCVdata.CurrentState.longitude_deg := WCV_Intervals.CurrentLongitude;
           
         --TODO finish else ladder for throwing an exception-----------------
         if not (m_DAIDALUSResponseServiceState.Heading_Min_deg <=
                   WCVdata.CurrentState.heading_deg and then 
                 WCVdata.CurrentState.heading_deg <= 
                   m_DAIDALUSResponseServiceState.Heading_Max_deg) 
         then
            raise Program_Error;
         else
            null;
         end if;
        
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
   
   procedure Process_MissionCommand_Message 
     (m_DAIDALUSResponseServiceState : in out Daidalus_Response_State;
      m_DAIDALUSResponseServiceConfig : Daidalus_Response_Configuration_Data;
      MissionCommandMessage : LMCP_Messages.MissionCommand) is
      SettingState : definitions.MissionCommand;
   begin
      if MissionCommandMessage.VehicleId = m_DAIDALUSResponseServiceConfig.
        VehicleID
      then
         SettingState.command_id := Common.Int64 (MissionCommandMessage.CommandId);
         SettingState.vehicle_id := Common.Int64 (MissionCommandMessage.VehicleId);
         case MissionCommandMessage.Status is
         when Pending => SettingState.status := definitions.Pending;
         when Approved => SettingState.status := definitions.Approved;
         when InProcess => SettingState.status := definitions.InProcess;
         when Executed => SettingState.status := definitions.Executed;
         when Cancelled => SettingState.status := definitions.Cancelled;
         end case;
         declare
            temp_val : definitions.VehicleActionList;
         begin
         
            for val of MissionCommandMessage.VehicleActionList loop
               declare
                  temp_atl : definitions.Associated_Tasks_List;
                  vehicleaction : definitions.VehicleAction;
               begin
                  for atl of val.AssociatedTaskList loop
                     MyVectorOfIntegers.Append (temp_atl, Common.Int64 (atl));
                  end loop;
                  vehicleaction.AssociatedTaskList := temp_atl;
                  MyVectorOfVehicleActions.Append (temp_val, vehicleaction);
               end;
            end loop;
            SettingState.vehicle_action_list := temp_val;
         end;
         SettingState.first_waypoint := Common.Int64 (MissionCommandMessage.FirstWaypoint);
         declare
            temp_waypoint_list : definitions.WaypointList;
         begin
            for waypointlist of MissionCommandMessage.WaypointList loop
               declare
                  temp_waypoint : definitions.Waypoint_info;
               begin
                  temp_waypoint.waypoint_number := Common.Int64 
                    (waypointlist.Number);
                  temp_waypoint.next_waypoint := Common.Int64 
                    (waypointlist.NextWaypoint);
                  temp_waypoint.speed := definitions.GroundSpeed_Type_mps 
                    (waypointlist.Speed);
                  case waypointlist.SpeedType is
                  when Airspeed => temp_waypoint.speed_type := 
                       definitions.Airspeed;
                  when Groundspeed => temp_waypoint.speed_type :=
                       definitions.Groundspeed;
                  end case;
                  temp_waypoint.climb_rate := definitions.VerticalSpeed_Type_mps 
                    (waypointlist.ClimbRate);
                  case waypointlist.TurnType is 
                  when TurnShort => temp_waypoint.turn_type := 
                       definitions.TurnShort;
                  when FlyOver => temp_waypoint.turn_type := 
                       definitions.FlyOver;
                  end case;
                  declare
                     val : definitions.VehicleActionList;
                  begin
                     for lmcp_val of waypointlist.VehicleActionList loop
                        declare
                           val_atl : definitions.Associated_Tasks_List;
                           vehicleaction : definitions.VehicleAction;
                        begin
                           for atl of lmcp_val.AssociatedTaskList loop
                              MyVectorOfIntegers.Append (val_atl, Common.Int64 
                                                         (atl));
                           end loop;
                           vehicleaction.AssociatedTaskList := val_atl;
                           MyVectorOfVehicleActions.Append (val, vehicleaction);
                        end;
                     end loop;
                     temp_waypoint.vehicle_action_list := val;
                  end;
                  temp_waypoint.contingency_waypoint_A := Common.Int64 
                    (waypointlist.ContingencyWaypointA);
                  temp_waypoint.contingency_waypoint_B := Common.Int64 
                    (waypointlist.ContingencyWaypointB);
                  declare
                     wp_atl : definitions.Associated_Tasks_List;
                  begin
                     for atl of waypointlist.AssociatedTasks loop
                        MyVectorOfIntegers.Append (wp_atl, Common.Int64 
                                                   (atl));
                     end loop;
                     temp_waypoint.associated_tasks := wp_atl;
                  end;
                  MyVectorOfWaypoints.Append (temp_waypoint_list, temp_waypoint);
               end;
            end loop;
            SettingState.waypoint_list := temp_waypoint_list;
         end;
         m_DAIDALUSResponseServiceState.MissionCommand := SettingState;
      end if;
      
   end Process_MissionCommand_Message;

end Daidalus_Response;
