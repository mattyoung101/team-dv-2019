/*
 * Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "music.h"

static void music_play_task(void *pvParameter){
    static const char *TAG = "MusicPlayer";
    music_note_t *song = (music_note_t*) pvParameter;
    size_t size = sizeof(song) / sizeof(music_note_t);

    for (int i = 0; i < size; i++){
        music_note_t note = song[i];
        ESP_LOGI(TAG, "Note: onoff(%d), freq(%d), time(%d)", note.type, note.frequency, note.time);
        vTaskDelay(pdMS_TO_TICKS(note.time));
    }
}

void music_play(music_note_t *song){
    xTaskCreate(music_play_task, "MusicPlayTask", 4096, song, configMAX_PRIORITIES - 4, NULL);
}