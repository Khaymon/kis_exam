#include <png.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

const size_t VERTEX_RADIUS = 60;
const size_t STEP_NUMBER = 4;

const int x_step[] = {0, 1, 0, -1};
const int y_step[] = {1, 0, -1, 0};

struct Point {
  int x;
  int y;

  Point() : x(0), y(0) {}
  Point(int x, int y) : x(x), y(y) {}

  bool operator==(const Point& lhs) const { return (x == lhs.x && y == lhs.y); }

  bool operator!=(const Point& lhs) const { return !(*this == lhs); }

  Point operator=(const Point& lhs) {
    x = lhs.x;
    y = lhs.y;
    return *this;
  }
};

struct RGB {
  char R;
  char G;
  char B;
};

bool InImage(int width, int height, int x, int y) {
  return x >= 0 && y >= 0 && x < width && y < height;
}

bool CheckVisited(std::vector<std::vector<bool> >& visited, int x, int y) {
  for (int i = 0; i < STEP_NUMBER; ++i) {
    for (int j = 0; j < VERTEX_RADIUS; ++j) {
      if (InImage(visited.size(), visited[0].size(), x + j * x_step[i],
                  y + j * y_step[i]) &&
          visited[x + j * x_step[i]][y + j * y_step[i]]) {
        return true;
      }
    }
  }
  return false;
}

bool IsBlack(std::vector<std::vector<RGB> >& matrix, int x, int y) {
  return InImage(matrix.size(), matrix[0].size(), x, y) &&
         matrix[x][y].R == 0 && matrix[x][y].B == 0 && matrix[x][y].G == 0;
}

double Square(double x) { return x * x; }

double SquaredDistance(Point& first, Point& second) {
  return Square(first.x - second.x) + Square(first.y - second.y);
}

bool AddVertex(Point point, std::vector<Point>& verts) {
  bool is_another_vertex = false;
  for (auto& vertex : verts) {
    if (SquaredDistance(vertex, point) <= VERTEX_RADIUS * VERTEX_RADIUS) {
      is_another_vertex = true;
      break;
    }
  }
  if (is_another_vertex) {
    return false;
  }

  verts.push_back(point);
  return true;
}

int GoWhileBlack(std::vector<std::vector<RGB> >& matrix, int x, int y,
                 int step) {
  int steps = 0;
  while (IsBlack(matrix, x + steps * x_step[step], y + steps * y_step[step])) {
    ++steps;
  }
  return steps;
}

// checks whether there is a path in current direction
bool TryDirection(std::vector<std::vector<RGB> >& matrix, int x, int y,
                  int step) {
  int previous_step = (step - 1 >= 0 ? step - 1 : STEP_NUMBER - 1);
  int steps = GoWhileBlack(matrix, x, y, step);
  if (steps == 0) {
    return false;
  }
  x += steps * x_step[step];
  y += steps * y_step[step];
  steps = GoWhileBlack(matrix, x, y, previous_step);
  if (steps == 0) {
    return false;
  }
  x += steps * x_step[step];
  y += steps * y_step[step];
  steps = GoWhileBlack(matrix, x, y, step);
  if (steps == 0) {
    return false;
  }
  return true;
}

Point GoBack(std::vector<std::vector<RGB> >& matrix, int x, int y,
             Point pattern) {
  while (IsBlack(matrix, x - pattern.x, y - pattern.y)) {
    x -= pattern.x;
    y -= pattern.y;
  }

  return Point(x + pattern.x, y + pattern.y);
}

void FollowLine(std::vector<std::vector<RGB> >& matrix, int x, int y,
                std::vector<Point>& verts,
                std::vector<std::vector<bool> >& visited,
                std::vector<std::pair<Point, Point> >& edges) {
  bool direction_changed = false;
  int repeats = 0;
  int begin_step = 0;
  int current_step = begin_step;
  int previous_step = -1;
  Point current_pattern;
  Point previous_position;
  Point pattern;

  while (true) {
    while (
        IsBlack(matrix, x + x_step[current_step], y + y_step[current_step]) &&
        !visited[x + x_step[current_step]][y + y_step[current_step]]) {
      x += x_step[current_step];
      y += y_step[current_step];
      visited[x][y] = true;
      current_pattern.x += x_step[current_step];
      current_pattern.y += y_step[current_step];
    }
    if (previous_step == -1) {
      previous_step = current_step;
      current_step = (current_step + 1) % STEP_NUMBER;
    } else {
      if (current_step == begin_step) {
        if (repeats == 0) {
          pattern = current_pattern;
          previous_position = Point(x, y);
        } else {
          if (current_pattern != pattern) {
            if (AddVertex(Point(x, y), verts)) {
              Point another_point = GoBack(matrix, x, y, pattern);
              AddVertex(another_point, verts);
              edges.emplace_back(Point(x, y), another_point);
            }
          }
        }
        ++repeats;
        current_pattern.x = 0;
        current_pattern.y = 0;
      }
      std::swap(current_step, previous_step);
      if (!IsBlack(matrix, x + x_step[current_step],
                   y + y_step[current_step])) {
        if (TryDirection(matrix, x, y, (current_step + 1) % 4)) {
          ++begin_step;
          previous_step = -1;
          current_step = begin_step;
          repeats = 0;
          direction_changed = true;
        } else {
          break;
        }
      }
    }
  }
}

void FindVertecies(std::vector<std::vector<RGB> >& matrix,
                   std::vector<Point>& verts,
                   std::vector<std::vector<bool> >& visited,
                   std::vector<std::pair<Point, Point> >& edges) {
  for (int i = 0; i < matrix.size(); ++i) {
    for (int j = 0; j < matrix[i].size(); ++j) {
      if (IsBlack(matrix, i, j) && !CheckVisited(visited, i, j)) {
        FollowLine(matrix, i, j, verts, visited, edges);
      }
    }
  }
}

int Area(Point a, Point b, Point c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool Intersects(std::pair<Point, Point>& first,
                std::pair<Point, Point>& second) {
  return Area(first.first, first.second, second.first) *
                 Area(first.first, first.second, second.second) <=
             0 &&
         Area(second.first, second.second, first.first) *
                 Area(second.first, second.second, first.second) <=
             0;
}

int main(int argc, char* argv[]) {
  if (argc == 1) {
    fprintf(stderr, "Can not find name of file\n");
    return 1;
  }

  int x, y;

  int width, height;
  png_byte color_type;
  png_byte bit_depth;

  png_structp png_ptr;
  png_infop info_ptr;
  int number_of_passes;
  png_bytep* row_pointers;

  char header[8];

  FILE* fp = fopen(argv[1], "rb");
  fread(header, 1, 8, fp);

  png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  info_ptr = png_create_info_struct(png_ptr);

  png_init_io(png_ptr, fp);
  png_set_sig_bytes(png_ptr, 8);

  png_read_info(png_ptr, info_ptr);

  width = png_get_image_width(png_ptr, info_ptr);
  height = png_get_image_height(png_ptr, info_ptr);
  color_type = png_get_color_type(png_ptr, info_ptr);
  bit_depth = png_get_bit_depth(png_ptr, info_ptr);

  number_of_passes = png_set_interlace_handling(png_ptr);
  png_read_update_info(png_ptr, info_ptr);

  row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
  for (y = 0; y < height; y++)
    row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png_ptr, info_ptr));

  png_read_image(png_ptr, row_pointers);
  fclose(fp);

  std::vector<std::vector<RGB> > image(width, std::vector<RGB>(height));
  for (y = 0; y < height; y++) {
    png_byte* row = row_pointers[y];
    for (x = 0; x < width; x++) {
      png_byte* ptr = &(row[x * 4]);
      image[x][y].R = ptr[0];
      image[x][y].G = ptr[1];
      image[x][y].B = ptr[2];
    }
  }
  std::vector<Point> verts;
  std::vector<std::pair<Point, Point> > edges;
  std::vector<std::vector<bool> > visited(width,
                                          std::vector<bool>(height, false));

  FindVertecies(image, verts, visited, edges);

  std::cout << "Vertecies count = " << verts.size() << std::endl;

  size_t intersection_count = 0;
  for (int i = 0; i < edges.size() - 1; ++i) {
    for (int j = i + 1; j < edges.size(); ++j) {
      if (Intersects(edges[i], edges[j])) {
        ++intersection_count;
      }
    }
  }

  std::cout << "Intersections count = " << intersection_count << std::endl;
}