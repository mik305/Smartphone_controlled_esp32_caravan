/*─────────────────────────────────────────────────────────────────────────────
 * levelling.h
 * Public API – moduł samopoziomowania przyczepy
 *
 * Copyright … 2025
 *───────────────────────────────────────────────────────────────────────────*/

#ifndef LEVELLING_H
#define LEVELLING_H

#include "esp_err.h"
#include <stdbool.h>

/*===========================================================================*/
/*  API do użycia w dowolnym pliku                                           */
/*===========================================================================*/

/* Uruchamia procedurę automatycznego poziomowania.
 * Jeśli zadanie już działa, wywołanie jest ignorowane.                      */
void level_start(void);

/* Natychmiast przerywa poziomowanie i zatrzymuje wszystkie siłowniki.
 * Jeśli zadanie nie jest aktywne - robi nic.                                */
void level_stop(void);

/* true  → aktualnie trwa poziomowanie
 * false → brak aktywnego zadania                                            */
bool level_busy(void);

/* Handler dla HTTP:  `/auto_level`
 *  • jeśli poziomowanie nie pracuje  →  startuje,
 *  • jeśli już pracuje               →  zatrzymuje.
 *  Zwraca zawsze ESP_OK, dzięki czemu strona dostaje HTTP 200.              */
/* ───────── NOWE  — u dołu pliku (przed #endif) ───────── */
esp_err_t level_http_start(void);   /* /auto_level_start  */
esp_err_t level_http_stop(void);    /* /auto_level_stop   */


#endif /* LEVELLING_H */
