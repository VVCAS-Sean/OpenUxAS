--  with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;
with definitions; use definitions;
with Ada.Numerics.Generic_Elementary_Functions;

package FlatEarth is

   package SafeFloat_Elementary_Functions is 
     new Ada.Numerics.Generic_Elementary_Functions (SafeFloat);
   use SafeFloat_Elementary_Functions;
      
   type FlatEarthObject is tagged private;
   
   procedure Initialize (feo :  out FlatEarthObject;
                        latitude_initial_rad : SafeFloat;
                        longitude_initial_rad : SafeFloat);
   
   --Convert Latitude Longitude to North East
   procedure ConvertLatitudeLongitude_rad_ToNorthEast_ft
     (feo : in out FlatEarthObject;
      latitude_rad : SafeFloat;
      longitude_rad : SafeFloat;
      north_ft : out SafeFloat;
      east_ft : out SafeFloat);

   procedure ConvertLatitudeLongitude_rad_ToNorthEast_m
     (feo : in out FlatEarthObject;
      latitude_rad : SafeFloat;
      longitude_rad : SafeFloat;
      north_m : out SafeFloat;
      east_m : out SafeFloat);
   
   procedure ConvertLatitudeLongitude_deg_ToNorthEast_m
     (feo : in out FlatEarthObject;
      latitude_deg : SafeFloat;
      longitude_deg : SafeFloat;
      north_m : out SafeFloat;
      east_m : out SafeFloat);
   
   procedure ConvertLatitudeLongitude_deg_ToNorthEast_ft
     (feo : in out FlatEarthObject;
      latitude_deg : SafeFloat;
      longitude_deg : SafeFloat;
      north_ft : out SafeFloat;
      east_ft : out SafeFloat);
   
   --Convert from North East to Latitude Longitude
   procedure ConvertNorthEast_m_ToLatitudeLongitude_rad 
     (feo : FlatEarthObject;
      north_m : SafeFloat;
      east_m : SafeFloat;
      latitude_rad : out SafeFloat;
      longitude_rad : out SafeFloat);
   
   procedure ConvertNorthEast_m_ToLatitudeLongitude_deg 
     (feo : FlatEarthObject;
      north_m : SafeFloat;
      east_m : SafeFloat;
      latitude_deg : out SafeFloat;
      longitude_deg : out SafeFloat);
   
   procedure ConvertNorthEast_ft_ToLatitudeLongitude_rad 
     (feo : FlatEarthObject;
      north_ft : SafeFloat;
      east_ft : SafeFloat;
      latitude_rad : out SafeFloat;
      longitude_rad : out SafeFloat);
   
   procedure ConvertNorthEast_ft_ToLatitudeLongitude_deg 
     (feo : FlatEarthObject;
      north_ft : SafeFloat;
      east_ft : SafeFloat;
      latitude_deg : out SafeFloat;
      longitude_deg : out SafeFloat);
   
   --Linear distances
   function GetLinearDistance_m_Lat1Long1_deg_To_Lat2Long2_deg 
     (feo : in out FlatEarthObject;
      latitude1_deg : SafeFloat;
      longitude1_deg : SafeFloat;
      latitude2_deg : SafeFloat;
      longitude2_deg : SafeFloat) return SafeFloat;
   
   function GetLinearDistance_m_Lat1Long1_rad_To_Lat2Long2_rad 
     (feo : in out FlatEarthObject;
      latitude1_rad : SafeFloat;
      longitude1_rad : SafeFloat;
      latitude2_rad : SafeFloat;
      longitude2_rad : SafeFloat) return SafeFloat;
   
   --Constants
   RadiusEquatorial_m : constant SafeFloat := 6_378_135.0;
   Flattening : constant SafeFloat := 3.352810664724998e-003;
   EccentricitySquared : constant SafeFloat := 6.694379990096503e-003;
   convertMetersToFeet : constant SafeFloat := 3.280839895;
   convertFeetToMeters : constant SafeFloat := 0.3048;
   convertDegreesToRadians : constant SafeFloat := 0.01745329251994;
   convertRadiansToDegrees : constant SafeFloat := 57.29577951308232;
   
private
   type FlatEarthObject is tagged record
      LatitudeInitial_rad : SafeFloat := 0.0;
      LongitudeInitial_rad : SafeFloat := 0.0;
      RadiusMeridional_m : SafeFloat := 0.0;
      RadiusTransverse_m : SafeFloat := 0.0;
      RadiusSmallCircleLatitude_m : SafeFloat := 0.0;
      isInitialized : Boolean := False;
   end record;
   
end FlatEarth;
