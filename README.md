# Modello ibrido per il movimento di un robot

Questo repository contiene un'implementazione del modello ibrido fra PRM e APF.

Il robot costruisce una Probabilistic Road Map per connettere la posizione iniziale con quella desiderata. Per connettere i nodi del grafo si utilizza il metodo epsilon-neighbors. Durante la costruzione della mappa, il robot tiene in considerazione gli ostacoli di cui è a conoscenza (quelli di colore grigio).

Connesso il punto iniziale a quello desiderato, il robot trova il percorso minimo per spostarsi fra questi.

Il robot si muove grazie ad APF. Il tragitto minimo è diviso in subgoal e ognuno di questi esercita un potenziale attrattivo che fa muovere il robot verso di questi. Raggiunto un subgoal, il robot passa al successivo. Gli ostacoli che rientrano nel raggio dei sensori del robot, applicano una forza repulsiva che respinge il robot da questi.

Durante la percorrenza del tragitto il robot può rilevare ostacoli di cui non era a conoscenza (quelli di colore rosso). Alcuni di questi ostacoli, potrebbero impedire la prosecuzione sul percorso minimo (il robot è finito in un minimo locale). Il robot avvia quindi una fase di replanning del percorso minimo tenendo in considerazione gli ostacoli nuovi incontrati.
Trovato il nuovo percorso minimo, il robot inizia a percorrerlo grazie ad APF.


## Parametri di configurazione
Nella funzione main() del codice sono presenti dei parametri che possono variare la configurazione del modello:

- **known_obs_number:** numero ostacoli conosciuti;

- **unknown_obs_number:** numero ostacoli sconosciuti;
   
- **eps:** costante epsilon con cui si trova gli epsilon-neighbors con la PRM;
   
- **num_iter:** numero minimo di nodi della PRM.

- **margin:** distanza per la quale un subgoal è considerato raggiunto;

- **step_size:** costante che indica il peso dell'anti-gradiente durante l'aggiornamento della posizione del robot;

- **attractive_delta:** costante utile al potenziale attrattivo Huber-like. Se il robot è più vicino al subgoal rispetto al valore di questa costante, allora sarà soggetto ad un potenziale qudratico, altrimenti sarà soggetto ad un potenziale distanza;

- **sensor_range:** raggio dei sensori del robot;

- **attractive_constant:** costante che indica il peso della forza attrattiva nel calcolo dell'anti-gradiente;

- **repulsive_constant:** costante che indica il peso della forza repulsiva nel calcolo dell'anti-gradiente;

- **security_distance:** distanza oltre la quale il robot non può avvicinarsi ad un ostacolo;

- **repulsive_potential_beta:** costante beta del potenziale repulsivo;

- **max_escape_local_minimum:** numero massimo di volte per cui il robot può uscire da un minimo locale.
