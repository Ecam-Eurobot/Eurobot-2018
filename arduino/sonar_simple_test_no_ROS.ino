#define TRIGG 12 // Broche TRIGGER
#define ECHO 13    // Broche ECHO
                                               // definition du Timeout
const long TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

float son= 340.0 / 1000; //vitesse du son dans l'air (mm/µs)

void setup() {

  pinMode(TRIGG, OUTPUT);  //Configuration des broches
  digitalWrite(TRIGG, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO, INPUT);

  Serial.begin(9600); //Démarrage de la liaison série
}
 
void loop() {
  
  digitalWrite(TRIGG, HIGH); // Lance une mesure de distance en envoyant 
  delayMicroseconds(10);  //une impulsion HIGH de 10µs sur la broche TRIGGER
  digitalWrite(TRIGG, LOW);
  
  int mesure = pulseIn(ECHO, HIGH, TIMEOUT); // Mesure le temps entre 
                                          // l'envoi de l'ultrason et sa réception

  float distance_mm = mesure / 2.0 * son; //calcul de la distance grâce au temps
                                      //on divise par 2 car le son fait un aller-retour
   
  Serial.print("Distance: "); //Affichage des résultats
  Serial.print(distance_mm);
  Serial.println("mm");

  delay(500); //temps entre chaque mesure (ms)
}
