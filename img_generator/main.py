from PIL import Image


def convert_image_to_binary_array(
    image_path, target_width=128, target_height=64
):
    """
    Конвертирует черно-белое изображение в бинарный массив (0 и 1).

    Args:
        image_path (str): Путь к файлу изображения.
        target_width (int): Ширина итогового массива в пикселях.
        target_height (int): Высота итогового массива в пикселях.

    Returns:
        list: Список списков, представляющий бинарный массив.
    """
    try:
        # Открываем изображение
        img = Image.open(image_path).convert(
            "L"
        )  # 'L' конвертирует в оттенки серого
        width, height = img.size

        # Рассчитываем размеры "квадратов" для сжатия
        x_step = width // target_width
        y_step = height // target_height

        # Вычисляем начальные координаты для центрирования
        x_offset = (width % target_width) // 2
        y_offset = (height % target_height) // 2

        binary_array = []
        for y in range(target_height):
            row = []
            for x in range(target_width):
                # Находим координаты центрального пикселя "квадрата"
                pixel_x = x_offset + x * x_step + x_step // 2
                pixel_y = y_offset + y * y_step + y_step // 2

                # Получаем значение пикселя
                pixel_value = img.getpixel((pixel_x, pixel_y))

                # Определяем порог для конвертации в 0 или 1 (например, 128)
                # Белый (255) станет 0, черный (0) станет 1
                binary_value = 1 if pixel_value < 128 else 0
                row.append(binary_value)
            binary_array.append(row)

        return binary_array

    except FileNotFoundError:
        print(f"Ошибка: Файл '{image_path}' не найден.")
        return None
    except Exception as e:
        print(f"Произошла ошибка: {e}")
        return None


def save_array_to_text_file(array, output_file_path):
    """
    Сохраняет бинарный массив в текстовый файл.
    """
    if array:
        try:
            with open(output_file_path, "w") as f:
                for row in array:
                    # Преобразуем список в строку с нужным форматом
                    f.write("{" + ", ".join(map(str, row)) + "},\n")
            print(f"Массив успешно сохранен в файле '{output_file_path}'.")
        except Exception as e:
            print(f"Ошибка при сохранении файла: {e}")


if __name__ == "__main__":
    input_image = "img.png"  # Замените на имя вашего файла
    output_text_file = "output_array.txt"

    # Конвертируем изображение в массив
    result_array = convert_image_to_binary_array(input_image)

    # Сохраняем массив в текстовый файл
    save_array_to_text_file(result_array, output_text_file)
