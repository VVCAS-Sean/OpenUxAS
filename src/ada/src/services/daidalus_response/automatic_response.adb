pragma Ada_2012;
with Ada.Containers;
package body automatic_response
with SPARK_Mode => On is

   ----------------------------
   -- Process_DAIDALUS_Bands --
   ----------------------------

   ----------------------------
   -- Process_DAIDALUS_Bands --
   ----------------------------

   procedure Process_DAIDALUS_Bands
     (Current_State                   :     state_parameters;
      Divert_State                    :    out state_parameters;
      DAIDALUS_Altitude_Bands         :     OrderedIntervalVector;
      DAIDALUS_Heading_Bands          :     OrderedIntervalVector;
      DAIDALUS_GroundSpeed_Bands      :     OrderedIntervalVector;
      Recovery_Altitude_Bands         :     OrderedIntervalVector;
      Recovery_Heading_Bands          :     OrderedIntervalVector;
      Recovery_GroundSpeed_Bands      :     OrderedIntervalVector;
      m_Vehicle_ID                    :     VehicleID_type;
      Intruders                       :     Intruder_info_Vector;
      DAIDALUS_Altitude_Zones         :     ZoneVector;
      DAIDALUS_Heading_Zones          :     ZoneVector;
      DAIDALUS_GroundSpeed_Zones      :     ZoneVector;
      m_isReady_to_Act                :     Boolean;
      m_Action_Time_Thresold_s        :     action_time_sec;
      m_Priority_Time_Threshold_s     :     priority_time_sec;
      m_Status                        : in out Status_Type;
      m_NextWaypoint                  :     Waypoint_info;
      Altitude_Max_m                  :     Altitude_Type_m;
      Altitude_Min_m                  :     Altitude_Type_m;
      Altitude_Interval_Buffer_m      :     Altitude_Buffer_Type_m;
      Heading_Max_deg                 :     Heading_Type_deg;
      Heading_Min_deg                 :     Heading_Type_deg;
      Heading_Interval_Buffer_deg     :     Heading_Buffer_Type_deg;
      GroundSpeed_Max_mps             :     GroundSpeed_Type_mps;
      GroundSpeed_Min_mps             :     GroundSpeed_Type_mps;
      GroundSpeed_Interval_Buffer_mps :     GroundSpeed_Buffer_Type_mps;
      Is_Tracking_Next_Waypoint       : in out Boolean;
      m_MissionCommand                : in out MissionCommand;
      RoW_ghost                       :    out ID_Type;
      ConflictResolutionList_ghost    :    out VehicleIDsVector;
      SendNewMissionCommand_ghost     :    out Boolean;
      Send_Divert_Action_Command_ghost :    out Boolean)
   is
      Conflict_Resolution_List : VehicleIDsVector;
      found_acceptable_action_flag : Boolean;
      isSafeToReturnToMissionFlag : Boolean;
      CheckState : state_parameters;
      CutPoint : Integer;
      PriorityStatus : Priority_Type := pStandard;
      RoW : ID_Type;
      m_StatusOld : Status_Type;
      --  vpimy: integer := 0;
   begin

      --initialize Divert state with the current state
      Divert_State := Current_State;
      --record status on entry
      m_StatusOld := m_Status;
      SendNewMissionCommand_ghost := False; --initialization
      Send_Divert_Action_Command_ghost := False; --initialization

      --Loop over the intruder information and populate a conflict resolution
      --list with those intruders that have a time to violation less than a
      --given threshold. Change priority of resolution based on intruder time to
      --violation against a separate given theshold.
      --  pragma Assert(Integer(MyVectorOfVehicleIDs.Capacity
      --                (Conflict_Resolution_List)) =
      --                  Integer(MyVectorOfIntruderInfo.Capacity(Intruders)));
      for I in MyVectorOfIntruderInfo.First_Index (Intruders) ..
        MyVectorOfIntruderInfo.Last_Index (Intruders) loop
         pragma Loop_Invariant (Integer (MyVectorOfVehicleIDs.Length
                               (Conflict_Resolution_List)) < I);
         if (not MyVectorOfIntruderInfo.Element (Intruders, I).
                Intruder_time_to_violation_isNan) and then
                 (MyVectorOfIntruderInfo.Element (Intruders, I).
                        Intruder_time_to_violation <=
                          m_Action_Time_Thresold_s)
         then

            MyVectorOfVehicleIDs.Append (Conflict_Resolution_List,
                                        MyVectorOfIntruderInfo.Element
                                          (Intruders, I).Intruder_ID);
         end if;
         if (not MyVectorOfIntruderInfo.Element (Intruders, I).
                Intruder_time_to_violation_isNan) and then
                 (MyVectorOfIntruderInfo.Element (Intruders, I).
                        Intruder_time_to_violation <=
                          m_Priority_Time_Threshold_s)
         then
            PriorityStatus := pHigh;
         end if;

      end loop;
      ConflictResolutionList_ghost := Conflict_Resolution_List;
      --here ends the first subprocedure

      --transition state to InConflict if the conflict resolution list is not
      --empty.  Prepare to set right of way vehicle by finding the intruder with
      --the lowest vehicle ID.
      if not MyVectorOfVehicleIDs.Is_Empty (Conflict_Resolution_List) then
         --initialize right-of-way vehicle with the highest possible ID
         RoW := ID_Type'Last;

         --set state to inconflict
         m_Status := InConflict;
         --  pragma Assert(NeedToResolveConflict(Conflict_Resolution_List));
         pragma Assert (if m_Status = InConflict then
                          NeedToResolveConflict (Conflict_Resolution_List));

         --if the conflict resolution list is not empty then either the altitude
         --, heading, or groundspeed bands has an interval that contains the
         --ownship with a time to violation less than the threshold by nature of
         --DAIDALUS detection and awareness
         pragma Assume (not (MyVectorOfIntervals.Is_Empty
                       (DAIDALUS_Altitude_Bands) and MyVectorOfIntervals.
                         Is_Empty (DAIDALUS_Heading_Bands) and
                         MyVectorOfIntervals.Is_Empty
                           (DAIDALUS_GroundSpeed_Bands)));
      else
         --set the right of way vehicle ID to the lowest possible to prevent it
         --from being reassigned.
         RoW := ID_Type'First;
         pragma Assert (not NeedToResolveConflict (Conflict_Resolution_List));
      end if;

      --loop over the intruder ID's setting the right of way to the intruder
      --with the lowest ID
      for I in MyVectorOfVehicleIDs.First_Index (Conflict_Resolution_List) ..
        MyVectorOfVehicleIDs.Last_Index (Conflict_Resolution_List) loop
         if MyVectorOfVehicleIDs.Element (Conflict_Resolution_List, I) < RoW
         then
            RoW := MyVectorOfVehicleIDs.Element (Conflict_Resolution_List, I);
         end if;
         pragma Loop_Invariant (for all J in MyVectorOfVehicleIDs.First_Index
                               (Conflict_Resolution_List) .. I => RoW <=
                                 MyVectorOfVehicleIDs.Element
                                   (Conflict_Resolution_List, J));
      end loop;

      RoW_ghost := RoW;

      --state machine implementation of responses
      case m_Status is
         --On mission, then do nothing
         when OnMission =>
            pragma Assert (Divert_State = Current_State);
            null;
         --when in conflict, do nothing if ownship is right of way, otherwise
         --divert
         when InConflict =>
            --  pragma Assert(m_Status = InConflict);
            pragma Assert (NeedToResolveConflict (Conflict_Resolution_List));
            if m_Vehicle_ID < RoW
            then
               m_Status := OnHold;
               pragma Assert (PerformedNoActionAsRoWVehicle
                             (Conflict_Resolution_List, RoW, m_Vehicle_ID,
                                Send_Divert_Action_Command_ghost,
                                SendNewMissionCommand_ghost));
            else
               pragma Assume (m_Vehicle_ID /= RoW);
               SetDivertState (DAIDALUS_Altitude_Bands, DAIDALUS_Heading_Bands,
                              DAIDALUS_GroundSpeed_Bands,
                              Recovery_Altitude_Bands, Recovery_Heading_Bands,
                              Recovery_GroundSpeed_Bands, Current_State,
                              Divert_State, found_acceptable_action_flag,
                              Altitude_Max_m, Altitude_Min_m,
                              Altitude_Interval_Buffer_m, Heading_Max_deg,
                              Heading_Min_deg, Heading_Interval_Buffer_deg,
                              GroundSpeed_Max_mps, GroundSpeed_Min_mps,
                              GroundSpeed_Interval_Buffer_mps,
                              PriorityStatus);
               Is_Tracking_Next_Waypoint := False;
               Send_Divert_Action_Command_ghost := True;
               -----------------------------------------------------------------
               --  SendDivertCommand(Divert_State, m_Vehicle_ID);
               -----------------------------------------------------------------
               m_Status := OnHold;
               pragma Assert (Diverted (Conflict_Resolution_List, RoW,
                             m_Vehicle_ID, Send_Divert_Action_Command_ghost,
                             SendNewMissionCommand_ghost));
            end if;
            --return to mission by sending updated missioncommand if safe-to-return
            --when previously on hold, otherwise continue mission if previously
            --on mission
         when OnHold =>
            if Is_Tracking_Next_Waypoint then
               m_Status := OnMission;
               pragma Assert (ContinuingLastCommand (Conflict_Resolution_List,
                             Send_Divert_Action_Command_ghost,
                             SendNewMissionCommand_ghost));
            else
               --handle checking for SafetoReturnToMission
               SafeToReturn
                 (DAIDALUS_Altitude_Bands, DAIDALUS_Heading_Bands,
                  DAIDALUS_GroundSpeed_Bands,
                  DAIDALUS_Altitude_Zones, DAIDALUS_Heading_Zones,
                  DAIDALUS_GroundSpeed_Zones, Current_State, CheckState,
                  m_NextWaypoint, m_MissionCommand, isSafeToReturnToMissionFlag)
                 ;
               if isSafeToReturnToMissionFlag
               then
                  --send a new mission command containing only the portion of
                  --previous mission command that has not yet be accomplished
                  --Waypoint number of -1 indicates no mission command being
                  --followed.
                  if not (m_NextWaypoint.waypoint_number = -1)
                  then
                     --set the 2nd waypoint as the first waypoint to better
                     --allow pathing from the autopilot.
                     CutPoint := MyVectorOfWaypoints.Find_Index
                       (m_MissionCommand.waypoint_list, m_NextWaypoint,
                        MyVectorOfWaypoints.
                          First_Index (m_MissionCommand.waypoint_list));
                     if not (CutPoint = MyVectorOfWaypoints.No_Index)
                     then
                        if not (CutPoint = MyVectorOfWaypoints.First_Index
                                (m_MissionCommand.waypoint_list))
                        then
                           MyVectorOfWaypoints.Delete_First (m_MissionCommand.
                                                              waypoint_list,
                                                            Ada.Containers.
                                                     Count_Type (CutPoint - 2));
                        end if;

                        --  MyVectorOfWaypoints.Delete(m_MissionCommand.
                        --                               waypoint_list,
                        --                         MyVectorOfWaypoints.First_Index
                        --                         (m_MissionCommand.waypoint_list),
                        --                             Cutpoint-1);
                        --  pragma Assert(MyVectorOfWaypoints.First_Index
                        --                (m_MissionCommand.waypoint_list) in
                        --                  MyVectorofWaypoint

                        --  for I in MyVectorOfWaypoints.First_Index
                        --    (m_MissionCommand.waypoint_list) .. (Cutpoint -2) loop
                        --     MyVectorofWaypoints.Delete(m_MissionCommand.
                        --                                  waypoint_list,
                        --                                MyVectorOfWaypoints.
                        --                                  First_Index
                        --                                    (m_MissionCommand.
                        --                                       waypoint_list));
                        --     pragma Loop_Invariant(MyVectorOfWaypoints.Last_Index
                        --                           (m_MissionCommand.
                        --                                waypoint_list) =
                        --                               MyVectorOfWaypoints.
                        --                                 Last_Index
                        --                             (m_MissionCommand.
                        --                                waypoint_list)'Loop_Entry
                        --                           - I);
                        --  end loop;

                     end if;
                     pragma Assume (2 in MyVectorOfWaypoints.First_Index
                                   (m_MissionCommand.waypoint_list) ..
                                     MyVectorOfWaypoints.Last_Index
                                       (m_MissionCommand.waypoint_list));
                     m_MissionCommand.first_waypoint := MyVectorOfWaypoints.
                       Element (m_MissionCommand.waypoint_list, 2).
                       waypoint_number;
                     -----------------------------------------------------------
                     --Send revised mission command
                     -----------------------------------------------------------
                     SendNewMissionCommand_ghost := True;
                     m_Status := OnMission;
                     Is_Tracking_Next_Waypoint := True;
                     pragma Assert (ReturnToMission (Conflict_Resolution_List,
                                   m_StatusOld, m_Status,
                                   Send_Divert_Action_Command_ghost,
                                   SendNewMissionCommand_ghost));
                  end if;
               else
                  pragma Assert (ContinuingLastCommand (Conflict_Resolution_List,
                                Send_Divert_Action_Command_ghost,
                                SendNewMissionCommand_ghost));
               end if;

            end if;
            pragma Assert (Divert_State = Current_State);

      end case;

      pragma Assert (if not NeedToResolveConflict (ConflictResolutionList_ghost)
                     then
                (Divert_State = Current_State and then (ContinuingLastCommand
                (ConflictResolutionList_ghost, Send_Divert_Action_Command_ghost,
                      SendNewMissionCommand_ghost) or ReturnToMission
                   (ConflictResolutionList_ghost, m_StatusOld, m_Status,
                    Send_Divert_Action_Command_ghost,
                    SendNewMissionCommand_ghost))) else
                  (PerformedNoActionAsRoWVehicle (ConflictResolutionList_ghost,
                   RoW_ghost, m_Vehicle_ID, Send_Divert_Action_Command_ghost,
                  SendNewMissionCommand_ghost) xor (
                  Diverted (ConflictResolutionList_ghost, RoW_ghost,
                m_Vehicle_ID, Send_Divert_Action_Command_ghost,
                SendNewMissionCommand_ghost))));

   end Process_DAIDALUS_Bands;

end automatic_response;
