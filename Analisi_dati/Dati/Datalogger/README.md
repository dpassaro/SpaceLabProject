Le acquisizioni dati precedenti il 02/11/2021 sono in formato non leggibile per i TTree di ROOT. Per questi file occorre:
- sostituire gli spazi con il carattere vuoto (nei dati GPS sono presenti degli spazi che fanno impazzire ROOT);
- sostituire eventuali ";;" con ";NA;" (la colonna del "Course over ground" può non funzionare)
- sostituire i ";" con " ".

Si può fare con un semplice editor di testo.
