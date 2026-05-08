## V - Search & Rescue

- Een bot valt uit en andere bots gaan naar deze bot toe of naar een beacon.
- Bij het uitvallen stuurt de bot een laatste signaal uit. Elke andere bot berekent hiermee de afstand tot de uitgevallen bot.

### Architectuur
- Aggregation?

### Communicatieopties

1. Elke robot stuurt zijn huidige positie en de afstand tot de uitgevallen bot naar een centrale server.  
   De server combineert deze informatie, bepaalt de positie van de uitgevallen bot en stuurt deze positie naar alle bots terug.  
   Vervolgens bepaalt elke bot zelfstandig zijn navigatiepad naar de doelpositie.

2. Gedecentraliseerde aanpak  
   Elke bot berekent zelfstandig de positie van de uitgevallen bot op basis van ontvangen signalen en gedeelde afstanden.  
   Daarna coördineren de bots onderling welke bots naar de locatie gaan.

### Extra feature-ideeën

- Wanneer een bot aankomt bij de uitgevallen bot:
  - tikt hij deze aan;
  - stopt hij met bewegen;
  - zet hij zijn licht aan;
  - en stuurt hij een bevestigingssignaal naar de server.

- Zodra alle bots zijn aangekomen, stuurt de server een `OK`-signaal terug.

---

## Object Verplaatsen

### Architectuur
- Collective Transport

### Communicatie
- Centralized Architecture

Er is een centrale server verbonden met alle bots en een camera die boven het speelveld hangt.

Wanneer een richting wordt aangegeven:
1. bepaalt de server welke posities de bots rondom het object moeten innemen;
2. stuurt de server de bots naar deze posities;
3. monitort de camera continu de beweging van het object.

Als het object afwijkt van de gewenste richting:
- wordt de bot met de meeste invloed op de correctie aangestuurd om van positie te veranderen;
- of wordt de kracht van specifieke bots aangepast.

---

## Vormen / Letters Creëren

### Architectuur
- Pattern Formation

### Communicatie
- Centralized Architecture

Er hangt een camera boven de bots die verbonden is met een centrale server.

De server beschikt over vooraf ingestelde vormen en weet welke posities de bots hiervoor moeten innemen.

Wanneer een vorm wordt gekozen:
1. berekent de server welke bots het dichtst bij welke doelposities staan;
2. worden de bots naar deze posities gestuurd;
3. controleert de camera of de formatie correct wordt gevormd.

---

## Pacman in Onbekend Terrein

### Mogelijke onderdelen
- Mapping
- Exploration
- Obstacle avoidance
- Pathfinding
- Multi-agent coördinatie
