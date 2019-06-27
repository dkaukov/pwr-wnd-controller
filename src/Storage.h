/**
 * MultiWii NG 0.1 - 2012
 * Permanent storage for onboard parameters
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

static uint8_t ver_magic = 11;

typedef struct nvr_entry nvr_entry_t;
struct nvr_entry{
  void *var;
  size_t size;
};

const nvr_entry_t nvr_entry[] = {
  {&ver_magic, sizeof(ver_magic)},
};
const uint8_t nvr_entry_cnt = sizeof(nvr_entry) / sizeof(nvr_entry_t);

void read_storage() {
  nvram_open(NVRAM_MODE_READ);
  for(uint8_t i = 0; i < nvr_entry_cnt; i++)
    nvram_read(nvr_entry[i].var, nvr_entry[i].size);
  nvram_close();
}

void write_storage() {
  nvram_open(NVRAM_MODE_WRITE);
  for(uint8_t i = 0; i < nvr_entry_cnt; i++)
    nvram_write(nvr_entry[i].var, nvr_entry[i].size);
  nvram_close();
}

void Storage_Init() {
  uint8_t test_val = ver_magic;
  nvram_open(NVRAM_MODE_READ); nvram_read(&test_val, 1); nvram_close();
  if (test_val != ver_magic)
    write_storage();
  else
    read_storage();
}

