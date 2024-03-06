pragma Ada_2012;
with Common; use Common;
package body FlatEarth is

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize
     (feo :  out FlatEarthObject;
      latitude_initial_rad : SafeFloat;
      longitude_initial_rad : SafeFloat)
   is
      DenominatorMeridional : SafeFloat;
      DenominatorTransverse : SafeFloat;
   begin
      if not feo.isInitialized
      then
         feo.LatitudeInitial_rad := latitude_initial_rad;
         feo.LongitudeInitial_rad := longitude_initial_rad;
         DenominatorMeridional := (1.0 - (EccentricitySquared *
                                   (Sin (latitude_initial_rad) ** 2.0))) **
                                   (3.0 / 2.0);
         pragma Assert (DenominatorMeridional > 0.0);
         feo.RadiusMeridional_m := (if DenominatorMeridional <= 0.0 then 0.0 else
                                   RadiusEquatorial_m * (1.0 -
                                    EccentricitySquared) / DenominatorMeridional);
         DenominatorTransverse := (1.0 - (Sin (latitude_initial_rad) ** 2.0)) **
           0.5;
         pragma Assert (DenominatorTransverse > 0.0);
         feo.RadiusTransverse_m := (if DenominatorTransverse <= 0.0 then 0.0 else
                                   RadiusEquatorial_m / DenominatorTransverse);
         feo.RadiusSmallCircleLatitude_m := feo.RadiusTransverse_m *
           Cos (latitude_initial_rad);
         feo.isInitialized := True;
      end if;

   end Initialize;

   -------------------------------------------------
   -- ConvertLatitudeLongitude_rad_ToNorthEast_ft --
   -------------------------------------------------

   procedure ConvertLatitudeLongitude_rad_ToNorthEast_ft
     (feo : in out FlatEarthObject;
      latitude_rad :     SafeFloat; longitude_rad : SafeFloat;
      north_ft     : out SafeFloat; east_ft : out SafeFloat)
   is
      north_m : SafeFloat;
      east_m : SafeFloat;
   begin
      if not feo.isInitialized then
         Initialize (feo, latitude_rad, longitude_rad);
      end if;
      north_m := feo.RadiusMeridional_m * (latitude_rad - feo.
                                             LatitudeInitial_rad);
      east_m := feo.RadiusSmallCircleLatitude_m * (longitude_rad -
                                              feo.LongitudeInitial_rad);
      north_ft := north_m * convertMetersToFeet;
      east_ft := east_m * convertMetersToFeet;
   end ConvertLatitudeLongitude_rad_ToNorthEast_ft;

   -----------------------------------------------
   -- CovertLatitudeLongitude_rat_ToNorthEast_m --
   -----------------------------------------------

   procedure ConvertLatitudeLongitude_rad_ToNorthEast_m
     (feo : in out FlatEarthObject;
      latitude_rad :     SafeFloat; longitude_rad : SafeFloat;
      north_m      : out SafeFloat; east_m : out SafeFloat)
   is
   begin
      if not feo.isInitialized then
         Initialize (feo, latitude_rad, longitude_rad);
      end if;
      north_m := feo.RadiusMeridional_m * (latitude_rad - feo.
                                             LatitudeInitial_rad);
      east_m := feo.RadiusSmallCircleLatitude_m * (longitude_rad -
                                                 feo.LongitudeInitial_rad);
   end ConvertLatitudeLongitude_rad_ToNorthEast_m;

   ------------------------------------------------
   -- ConvertLatitudeLongitude_deg_ToNorthEast_m --
   ------------------------------------------------

   procedure ConvertLatitudeLongitude_deg_ToNorthEast_m
     (feo : in out FlatEarthObject;
      latitude_deg :     SafeFloat; longitude_deg : SafeFloat;
      north_m      : out SafeFloat; east_m : out SafeFloat)
   is
      latitude_rad : SafeFloat := latitude_deg * convertDegreesToRadians;
      longitude_rad : SafeFloat := longitude_deg * convertDegreesToRadians;
   begin
      if not feo.isInitialized then
         Initialize (feo, latitude_rad, longitude_rad);
      end if;
      north_m := feo.RadiusMeridional_m * (latitude_rad - feo.
                                             LatitudeInitial_rad);
      east_m := feo.RadiusSmallCircleLatitude_m * (longitude_rad -
                                                 feo.LongitudeInitial_rad);
   end ConvertLatitudeLongitude_deg_ToNorthEast_m;

   -------------------------------------------------
   -- ConvertLatitudeLongitude_deg_ToNorthEast_ft --
   -------------------------------------------------

   procedure ConvertLatitudeLongitude_deg_ToNorthEast_ft
     (feo : in out FlatEarthObject;
      latitude_deg :     SafeFloat; longitude_deg : SafeFloat;
      north_ft     : out SafeFloat; east_ft : out SafeFloat)
   is
      latitude_rad : SafeFloat := latitude_deg * convertDegreesToRadians;
      longitude_rad : SafeFloat := longitude_deg * convertDegreesToRadians;
      north_m : SafeFloat;
      east_m : SafeFloat;
   begin
      if not feo.isInitialized then
         Initialize (feo, latitude_rad, longitude_rad);
      end if;
      north_m := feo.RadiusMeridional_m * (latitude_rad - feo.
                                             LatitudeInitial_rad);
      east_m := feo.RadiusSmallCircleLatitude_m * (longitude_rad -
                                                 feo.LongitudeInitial_rad);
      north_ft := north_m * convertMetersToFeet;
      east_ft := east_m * convertMetersToFeet;

   end ConvertLatitudeLongitude_deg_ToNorthEast_ft;

   ------------------------------------------------
   -- ConvertNorthEast_m_ToLatitudeLongitude_rad --
   ------------------------------------------------

   procedure ConvertNorthEast_m_ToLatitudeLongitude_rad
     (feo : FlatEarthObject;
      north_m       : SafeFloat; east_m : SafeFloat;
      latitude_rad : out SafeFloat;
      longitude_rad : out SafeFloat)
   is
   begin
      pragma Assert (feo.RadiusMeridional_m > 0.0);
      latitude_rad := (if feo.RadiusMeridional_m <= 0.0 then 0.0 else ((north_m
                       / feo.RadiusMeridional_m) + feo.LatitudeInitial_rad));
      pragma Assert (feo.RadiusSmallCircleLatitude_m > 0.0);
      longitude_rad := (if feo.RadiusSmallCircleLatitude_m <= 0.0 then 0.0 else
                          ((east_m / feo.RadiusSmallCircleLatitude_m) +
                               feo.LongitudeInitial_rad));
   end ConvertNorthEast_m_ToLatitudeLongitude_rad;

   ------------------------------------------------
   -- ConvertNorthEast_m_ToLatitudeLongitude_deg --
   ------------------------------------------------

   procedure ConvertNorthEast_m_ToLatitudeLongitude_deg
     (feo : FlatEarthObject;
      north_m       : SafeFloat; east_m : SafeFloat;
      latitude_deg : out SafeFloat;
      longitude_deg : out SafeFloat)
   is
   begin
      pragma Assert (feo.RadiusMeridional_m > 0.0);
      latitude_deg := (if feo.RadiusMeridional_m <= 0.0 then 0.0 else ((north_m
                       / feo.RadiusMeridional_m) + feo.LatitudeInitial_rad)) *
                       convertRadiansToDegrees;
      pragma Assert (feo.RadiusSmallCircleLatitude_m > 0.0);
      longitude_deg := (if feo.RadiusSmallCircleLatitude_m <= 0.0 then 0.0 else
                          ((east_m / feo.RadiusSmallCircleLatitude_m) +
                               feo.LongitudeInitial_rad)) *
            convertRadiansToDegrees;

   end ConvertNorthEast_m_ToLatitudeLongitude_deg;

   -------------------------------------------------
   -- ConvertNorthEast_ft_ToLatitudeLongitude_rad --
   -------------------------------------------------

   procedure ConvertNorthEast_ft_ToLatitudeLongitude_rad
     (feo : FlatEarthObject;
      north_ft :     SafeFloat; east_ft : SafeFloat;
      latitude_rad : out SafeFloat;
      longitude_rad : out SafeFloat)
   is
      north_m : SafeFloat := north_ft * convertFeetToMeters;
      east_m : SafeFloat := east_ft * convertFeetToMeters;
   begin
      pragma Assert (feo.RadiusMeridional_m > 0.0);
      latitude_rad := (if feo.RadiusMeridional_m <= 0.0 then 0.0 else ((north_m
                       / feo.RadiusMeridional_m) + feo.LatitudeInitial_rad));
      pragma Assert (feo.RadiusSmallCircleLatitude_m > 0.0);
      longitude_rad := (if feo.RadiusSmallCircleLatitude_m <= 0.0 then 0.0 else
                          ((east_m / feo.RadiusSmallCircleLatitude_m) +
                               feo.LongitudeInitial_rad));
   end ConvertNorthEast_ft_ToLatitudeLongitude_rad;

   -------------------------------------------------
   -- ConvertNorthEast_ft_ToLatitudeLongitude_deg --
   -------------------------------------------------

   procedure ConvertNorthEast_ft_ToLatitudeLongitude_deg
     (feo : FlatEarthObject;
      north_ft :     SafeFloat; east_ft : SafeFloat;
      latitude_deg : out SafeFloat;
      longitude_deg : out SafeFloat)
   is
      north_m : SafeFloat := north_ft * convertFeetToMeters;
      east_m : SafeFloat := east_ft * convertFeetToMeters;
      latitude_rad : SafeFloat;
      longitude_rad : SafeFloat;
   begin
      pragma Assert (feo.RadiusMeridional_m > 0.0);
      latitude_rad := (if feo.RadiusMeridional_m <= 0.0 then 0.0 else ((north_m
                       / feo.RadiusMeridional_m) + feo.LatitudeInitial_rad));
      latitude_deg := latitude_rad * convertRadiansToDegrees;
      pragma Assert (feo.RadiusSmallCircleLatitude_m > 0.0);
      longitude_rad := (if feo.RadiusSmallCircleLatitude_m <= 0.0 then 0.0 else
                          ((east_m / feo.RadiusSmallCircleLatitude_m) +
                               feo.LongitudeInitial_rad));
      longitude_deg := longitude_rad * convertRadiansToDegrees;

   end ConvertNorthEast_ft_ToLatitudeLongitude_deg;

   --------------------------------------------------------
   -- GetLinearDistance_m_Lat1Long1_deg_To_Lat2Long2_deg --
   --------------------------------------------------------

   function GetLinearDistance_m_Lat1Long1_deg_To_Lat2Long2_deg
     (feo : in out FlatEarthObject;
      latitude1_deg : SafeFloat; longitude1_deg : SafeFloat;
      latitude2_deg : SafeFloat; longitude2_deg : SafeFloat) return
     SafeFloat
   is
      north1_m : SafeFloat := 0.0;
      east1_m : SafeFloat := 0.0;
      north2_m : SafeFloat := 0.0;
      east2_m : SafeFloat := 0.0;
      ReturnValue : SafeFloat;
   begin
      ConvertLatitudeLongitude_deg_ToNorthEast_m (feo,
                                                 latitude1_deg, longitude1_deg,
                                                 north1_m, east1_m);
      ConvertLatitudeLongitude_deg_ToNorthEast_m (feo,
                                                 latitude2_deg, longitude2_deg,
                                                 north2_m, east2_m);
      ReturnValue := (((north2_m - north1_m) ** 2.0) + ((east2_m - east1_m) **
                        2.0)) ** 0.5;
      return ReturnValue;
   end GetLinearDistance_m_Lat1Long1_deg_To_Lat2Long2_deg;

   --------------------------------------------------------
   -- GetLinearDistance_m_Lat1Long1_rad_To_Lat2Long2_rad --
   --------------------------------------------------------

   function GetLinearDistance_m_Lat1Long1_rad_To_Lat2Long2_rad
     (feo : in out FlatEarthObject;
      latitude1_rad : SafeFloat; longitude1_rad : SafeFloat;
      latitude2_rad : SafeFloat; longitude2_rad : SafeFloat) return
     SafeFloat
   is
      north1_m : SafeFloat := 0.0;
      east1_m : SafeFloat := 0.0;
      north2_m : SafeFloat := 0.0;
      east2_m : SafeFloat := 0.0;
      ReturnValue : SafeFloat;
   begin
      ConvertLatitudeLongitude_rad_ToNorthEast_m (feo,
                                                 latitude1_rad, longitude1_rad,
                                                 north1_m, east1_m);
      ConvertLatitudeLongitude_rad_ToNorthEast_m (feo,
                                                 latitude2_rad, longitude2_rad,
                                                 north2_m, east2_m);
      ReturnValue := (((north2_m - north1_m) ** 2.0) + ((east2_m - east1_m) **
                        2.0)) ** 0.5;
      return ReturnValue;
   end GetLinearDistance_m_Lat1Long1_rad_To_Lat2Long2_rad;

end FlatEarth;
