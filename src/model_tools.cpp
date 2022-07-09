#include <random>
#include "model_tools.h"
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <CGAL/approximated_offset_2.h>
#include <CGAL/point_generators_3.h>

Surface_mesh read_model(const fs::path& v_path)
{
    Surface_mesh mesh;
	if(!fs::exists(v_path))
	{
		std::cout << "Cannot find " << v_path << std::endl;
		throw;
	}
	if(!CGAL::IO::read_PLY(v_path.string(),mesh))
	{
		std::cout << "Cannot load ply model, try to repair it"<< std::endl;
		CGAL::Polyhedron_3<K> poly;
		CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(v_path.string(),poly);
		std::cout << "Try to load it as a polyhedron. Get " << poly.size_of_vertices() << " vertices and " << poly.size_of_facets() << " faces"<< std::endl;

		CGAL::IO::write_PLY("temp_read.ply",poly);
		if(!CGAL::IO::read_PLY("temp_read.ply",mesh))
		{
			std::cout << "Still cannot load it. Try to mannually repair it"<< std::endl;
			throw;
		}
	}
	std::cout << "Load done. Get "<<mesh.num_vertices()<<" vertices and "<<mesh.num_faces()<<" faces" << std::endl;

    return mesh;
}

bool read_model(const fs::path& v_path,Polyhedron_3& v_mesh)
{
	if(!fs::exists(v_path))
	{
		std::cout << "Cannot find " << v_path << std::endl;
		throw;
	}
	if(!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(v_path.string(),v_mesh))
	{
		std::cout << "Cannot load ply model, try to repair it"<< std::endl;
		throw;
	}
	std::cout << "Load done. Get "<<v_mesh.size_of_vertices()<<" vertices and "<<v_mesh.size_of_facets()<<" faces" << std::endl;

    return true;
}

bool read_model(const fs::path& v_path, std::vector<Point_3>& v_points, std::vector<Vector_3>& v_normals, std::vector<std::array<int,3>>& v_face_indices,
	std::vector<CGAL::Triangle_3<K>>& v_faces)
{
	struct memory_buffer : public std::streambuf
	{
		char* p_start{ nullptr };
		char* p_end{ nullptr };
		size_t size;

		memory_buffer(char const* first_elem, size_t size)
			: p_start(const_cast<char*>(first_elem)), p_end(p_start + size), size(size)
		{
			setg(p_start, p_start, p_end);
		}

		pos_type seekoff(off_type off, std::ios_base::seekdir dir, std::ios_base::openmode which) override
		{
			if (dir == std::ios_base::cur) gbump(static_cast<size_t>(off));
			else setg(p_start, (dir == std::ios_base::beg ? p_start : p_end) + off, p_end);
			return gptr() - p_start;
		}

		pos_type seekpos(pos_type pos, std::ios_base::openmode which) override
		{
			return seekoff(pos, std::ios_base::beg, which);
		}
	};

	struct memory_stream : virtual memory_buffer, public std::istream
	{
		memory_stream(char const* first_elem, size_t size)
			: memory_buffer(first_elem, size), std::istream(static_cast<std::streambuf*>(this)) {}
	};

	std::unique_ptr<std::istream> file_stream;
	std::vector<uint8_t> byte_buffer;

	try
	{
		std::ifstream dummy_file(v_path.string(), std::ios::binary);

		if (!dummy_file.is_open())
			throw std::runtime_error("could not open binary ifstream to path " + v_path.string());
		dummy_file.seekg(0, std::ios::end);
		size_t sizeBytes = dummy_file.tellg();
		std::cout << "The file is " << sizeBytes / 1024 / 1024 << " MB" << std::endl;
		if (sizeBytes > 1024 * 1024 * 1024)
		{
			file_stream.reset(new std::ifstream(v_path.string(), std::ios::binary));
		}
		else
		{
			dummy_file.seekg(0, std::ios::beg);
			byte_buffer.resize(sizeBytes);
			if (dummy_file.read((char*)byte_buffer.data(), sizeBytes));
			file_stream.reset(new memory_stream((char*)byte_buffer.data(), byte_buffer.size()));
		}
			
		if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + v_path.string());

		tinyply::PlyFile file;
		if(!file.parse_header(*file_stream))
		{
			std::cerr << "Error parsing the header";
			throw;
		}

		//std::cout << "\t[ply_header] Type: " << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
		//for (const auto & c : file.get_comments()) std::cout << "\t[ply_header] Comment: " << c << std::endl;
		//for (const auto & c : file.get_info()) std::cout << "\t[ply_header] Info: " << c << std::endl;

		for (const auto& e : file.get_elements())
		{
			//std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")" << std::endl;
			for (const auto& p : e.properties)
			{
				//std::cout << "\t[ply_header] \tproperty: " << p.name << " (type=" << tinyply::PropertyTable[p.propertyType].str << ")";
				//if (p.isList) std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str << ")";
				//std::cout << std::endl;
			}
		}

		std::shared_ptr<tinyply::PlyData> vertices, normals, colors, texcoords, faces, tripstrip;
		bool has_normals = true;
		try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
		catch (const std::exception& e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		try { normals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }); }
		catch (const std::exception& e)
		{
			has_normals = false;
			std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		try { colors = file.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" }); }
		catch (const std::exception& e)
		{
			//std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		try { colors = file.request_properties_from_element("vertex", { "r", "g", "b", "a" }); }
		catch (const std::exception& e)
		{
			//std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		try { texcoords = file.request_properties_from_element("vertex", { "u", "v" }); }
		catch (const std::exception& e)
		{
			//std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
		catch (const std::exception& e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		// Tristrips must always be read with a 0 list size hint (unless you know exactly how many elements
		// are specifically in the file, which is unlikely); 
		try { tripstrip = file.request_properties_from_element("tristrips", { "vertex_indices" }, 0); }
		catch (const std::exception& e)
		{
			//std::cerr << "tinyply exception: " << e.what() << std::endl;
		}

		file.read(*file_stream);

		//if (vertices)   std::cout << "\tRead " << vertices->count  << " total vertices "<< std::endl;
		//if (normals)    std::cout << "\tRead " << normals->count   << " total vertex normals " << std::endl;
		//if (colors)     std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;
		//if (texcoords)  std::cout << "\tRead " << texcoords->count << " total vertex texcoords " << std::endl;
		//if (faces)      std::cout << "\tRead " << faces->count     << " total faces (triangles) " << std::endl;
		//if (tripstrip)  std::cout << "\tRead " << (tripstrip->buffer.size_bytes() / tinyply::PropertyTable[tripstrip->t].stride) << " total indicies (tristrip) " << std::endl;

		//LOG(INFO) << "Read done. Generating the vector";
		v_points.resize(vertices->count);
		v_normals.resize(vertices->count);
		v_faces.resize(faces->count);
		v_face_indices.resize(faces->count);
		
		if (vertices->t == tinyply::Type::FLOAT32)
		{
			#pragma omp parallel for
			for (int i = 0;i < vertices->count;++i)
			{
				v_points[i] = Point_3(
					((float*)vertices->buffer.get())[i * 3 + 0],
					((float*)vertices->buffer.get())[i * 3 + 1],
					((float*)vertices->buffer.get())[i * 3 + 2]
				);
			}
		}
		else if (vertices->t == tinyply::Type::FLOAT64)
		{
			#pragma omp parallel for
			for (int i = 0;i < vertices->count;++i)
			{
				v_points[i] = Point_3(
					((double*)vertices->buffer.get())[i * 3 + 0],
					((double*)vertices->buffer.get())[i * 3 + 1],
					((double*)vertices->buffer.get())[i * 3 + 2]
				);
			}
		}
		else
			throw;

		if(has_normals)
		{
			if (normals->t == tinyply::Type::FLOAT32)
			{
#pragma omp parallel for
				for (int i = 0;i < normals->count;++i)
				{
					v_normals[i] = Vector_3(
						((float*)normals->buffer.get())[i * 3 + 0],
						((float*)normals->buffer.get())[i * 3 + 1],
						((float*)normals->buffer.get())[i * 3 + 2]
					);
				}
			}
			else if (normals->t == tinyply::Type::FLOAT64)
			{
#pragma omp parallel for
				for (int i = 0;i < vertices->count;++i)
				{
					v_normals[i] = Vector_3(
						((double*)normals->buffer.get())[i * 3 + 0],
						((double*)normals->buffer.get())[i * 3 + 1],
						((double*)normals->buffer.get())[i * 3 + 2]
					);
				}
			}
			else
				throw;
		}
		
		if (faces->t == tinyply::Type::UINT32)
		{
#pragma omp parallel for
			for (int i = 0;i < faces->count;++i)
			{
				v_faces[i] = CGAL::Triangle_3<K>(
					v_points[((unsigned int*)faces->buffer.get())[i * 3 + 0]],
					v_points[((unsigned int*)faces->buffer.get())[i * 3 + 1]],
					v_points[((unsigned int*)faces->buffer.get())[i * 3 + 2]]);
				v_face_indices[i] = {
					(int)(((unsigned int*)faces->buffer.get())[i * 3 + 0]) ,
					(int)(((unsigned int*)faces->buffer.get())[i * 3 + 1]),
					(int)(((unsigned int*)faces->buffer.get())[i * 3 + 2]) };
			}
		}
		else if(faces->t == tinyply::Type::INT32)
#pragma omp parallel for
			for (int i = 0;i < faces->count;++i)
			{
				v_faces[i] = CGAL::Triangle_3<K>(
					v_points[((int*)faces->buffer.get())[i * 3 + 0]],
					v_points[((int*)faces->buffer.get())[i * 3 + 1]],
					v_points[((int*)faces->buffer.get())[i * 3 + 2]]);
				v_face_indices[i] = { ((int*)faces->buffer.get())[i * 3 + 0] ,((int*)faces->buffer.get())[i * 3 + 1], ((int*)faces->buffer.get())[i * 3 + 2] };
			}
		else
			throw;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
	}

	return true;
}

/*
Some useful function
*/

std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>> load_obj(
	const std::string& v_mesh_name, bool v_log, const std::string& v_mtl_dir)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string warn;
	std::string err;
	bool ret;

	if (v_log)
		std::cout << "Read mesh " << v_mesh_name << " with TinyOBJLoader" << std::endl;
	ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
	                       v_mesh_name.c_str(), v_mtl_dir.c_str(), true);

	if (!warn.empty())
	{
		std::cout << warn << std::endl;
	}
	if (!err.empty())
	{
		std::cerr << err << std::endl;
	}
	if (!ret)
	{
		exit(1);
	}
	if (v_log)
	{
		int num_face = 0;
		for (const auto& shape : shapes)
			num_face += shape.mesh.indices.size() / 3;
		std::cout << "Read with " << attrib.vertices.size() / 3 << " vertices," << attrib.normals.size() / 3 << " normals," <<
			num_face << " faces" << std::endl;
	}
	for (auto & item_material : materials)
	{
		if(item_material.diffuse_texname!="")
		{
			item_material.diffuse_texname = (boost::filesystem::path(v_mtl_dir) / item_material.diffuse_texname).string();
		}
	}
	return {attrib, shapes, materials};
}

Proxy load_footprint(const std::string& v_footprint_path)
{
	std::ifstream file_in(v_footprint_path);
	if (!file_in.is_open()) {
		std::cout << "No such file " << v_footprint_path << std::endl;
		return Proxy();
	}
	Proxy proxy;
	std::string line;
	std::getline(file_in, line);
	std::vector<std::string> tokens;
	boost::split(tokens, line, boost::is_any_of(" "));
	proxy.height = std::atof(tokens[1].c_str());
	std::getline(file_in, line);

	tokens.clear();
	boost::split(tokens, line, boost::is_any_of(" "));
	for (int i = 0; i < tokens.size(); i += 2)
	{
		if(tokens[i]=="")
			break;
		float x = std::atof(tokens[i].c_str());
		float y = std::atof(tokens[i+1].c_str());
		proxy.polygon.push_back(CGAL::Point_2<K>(x, y));
	}
	
	file_in.close();
	return proxy;
}

bool WriteMat(const std::string& filename, const std::vector<tinyobj::material_t>& materials)
{
	FILE* fp = fopen(filename.c_str(), "w");
	if (!fp)
	{
		fprintf(stderr, "Failed to open file [ %s ] for write.\n", filename.c_str());
		return false;
	}

	for (size_t i = 0; i < materials.size(); i++)
	{
		tinyobj::material_t mat = materials[i];

		fprintf(fp, "newmtl %s\n", mat.name.c_str());
		fprintf(fp, "Ka %f %f %f\n", mat.ambient[0], mat.ambient[1], mat.ambient[2]);
		fprintf(fp, "Kd %f %f %f\n", mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
		fprintf(fp, "Ks %f %f %f\n", mat.specular[0], mat.specular[1], mat.specular[2]);
		fprintf(fp, "Kt %f %f %f\n", mat.transmittance[0], mat.specular[1], mat.specular[2]);
		fprintf(fp, "Ke %f %f %f\n", mat.emission[0], mat.emission[1], mat.emission[2]);
		fprintf(fp, "d %f\n", 1);
		fprintf(fp, "Ns %f\n", mat.shininess);
		fprintf(fp, "Ni %f\n", mat.ior);
		fprintf(fp, "illum %d\n", mat.illum);
		if (mat.diffuse_texname.size() > 2)
			fprintf(fp, "map_Kd %s\n", mat.diffuse_texname.c_str());
		if (mat.ambient_texname.size() > 2)
			fprintf(fp, "map_Ka %s\n", mat.ambient_texname.c_str());
		//fprintf(fp, "map_Kd %s\n", mat.diffuse_texname.c_str());
		fprintf(fp, "\n");
		// @todo { texture }
	}

	fclose(fp);

	return true;
}

bool write_obj(const std::string& filename, const tinyobj::attrib_t& attributes,
               const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials)
{
	FILE* fp = fopen(filename.c_str(), "w");
	if (!fp)
	{
		fprintf(stderr, "Failed to open file [ %s ] for write.\n", filename.c_str());
		return false;
	}

	boost::filesystem::path output_file(filename);
	std::string basename = output_file.filename().stem().string() + ".mtl";
	std::string material_filename = (output_file.parent_path()/ basename).string();

	int prev_material_id = -1;

	fprintf(fp, "mtllib %s\n\n", basename.c_str());

	// facevarying vtx
	for (size_t k = 0; k < attributes.vertices.size(); k += 3)
	{
		fprintf(fp, "v %f %f %f\n",
		        attributes.vertices[k + 0],
		        attributes.vertices[k + 1],
		        attributes.vertices[k + 2]);
	}

	fprintf(fp, "\n");

	// facevarying normal
	for (size_t k = 0; k < attributes.normals.size(); k += 3)
	{
		fprintf(fp, "vn %f %f %f\n",
		        attributes.normals[k + 0],
		        attributes.normals[k + 1],
		        attributes.normals[k + 2]);
	}

	fprintf(fp, "\n");

	// facevarying texcoord
	for (size_t k = 0; k < attributes.texcoords.size(); k += 2)
	{
		fprintf(fp, "vt %f %f\n",
		        attributes.texcoords[k + 0],
		        attributes.texcoords[k + 1]);
	}

	for (size_t i = 0; i < shapes.size(); i++)
	{
		fprintf(fp, "\n");

		if (shapes[i].name.empty())
		{
			fprintf(fp, "g %s\n", std::to_string(i));
		}
		else
		{
			//fprintf(fp, "use %s\n", shapes[i].name.c_str());
			fprintf(fp, "g %s\n", shapes[i].name.c_str());
		}

		bool has_vn = false;
		bool has_vt = false;
		// Assumes normals and textures are set shape-wise.
		if (shapes[i].mesh.indices.size() > 0)
		{
			has_vn = shapes[i].mesh.indices[0].normal_index != -1;
			has_vt = shapes[i].mesh.indices[0].texcoord_index != -1;
		}

		// face
		int face_index = 0;
		for (size_t k = 0; k < shapes[i].mesh.indices.size(); k += shapes[i].mesh.num_face_vertices[face_index++])
		{
			// Check Materials
			int material_id = shapes[i].mesh.material_ids[face_index];
			if (material_id != prev_material_id && material_id >= 0)
			{
				std::string material_name = materials[material_id].name;
				fprintf(fp, "usemtl %s\n", material_name.c_str());
				prev_material_id = material_id;
			}

			unsigned char v_per_f = shapes[i].mesh.num_face_vertices[face_index];
			// Imperformant, but if you want to have variable vertices per face, you need some kind of a dynamic loop.
			fprintf(fp, "f");
			for (int l = 0; l < v_per_f; l++)
			{
				const tinyobj::index_t& ref = shapes[i].mesh.indices[k + l];
				if (has_vn && has_vt)
				{
					// v0/t0/vn0
					fprintf(fp, " %d/%d/%d", ref.vertex_index + 1, ref.texcoord_index + 1, ref.normal_index + 1);
					continue;
				}
				if (has_vn && !has_vt)
				{
					// v0//vn0
					fprintf(fp, " %d//%d", ref.vertex_index + 1, ref.normal_index + 1);
					continue;
				}
				if (!has_vn && has_vt)
				{
					// v0/vt0
					fprintf(fp, " %d/%d", ref.vertex_index + 1, ref.texcoord_index + 1);
					continue;
				}
				if (!has_vn && !has_vt)
				{
					// v0 v1 v2
					fprintf(fp, " %d", ref.vertex_index + 1);
					continue;
				}
			}
			fprintf(fp, "\n");
		}
	}

	bool ret = WriteMat(material_filename, materials);

	fclose(fp);
	int num_face = 0;
	for (const auto& shape : shapes)
		num_face += shape.mesh.indices.size() / 3;
	std::cout << "Write with " << attributes.vertices.size() / 3 << " vertices," << attributes.normals.size() / 3 << " normals," <<
		num_face << " faces" << std::endl;
	return 1;
}

void fix_mtl_from_unreal(const std::string& filename)
{
	std::ifstream f_in(filename);
	std::string buffer;

	if (!f_in.is_open())
	{
		std::cout << "No such file " << filename << std::endl;
		return;
	}

	int material_idx = -1;
	while (!f_in.eof())
	{
		std::string line;
		std::getline(f_in, line);
		if (line.find("newmtl") != line.npos)
		{
			material_idx += 1;
			buffer += "\n newmtl material_" + std::to_string(material_idx);
		}
		else
		{
			if (material_idx == -1) //First line
				continue;
			else
				buffer += "\n" + line;
		}
	}
	f_in.close();
	std::ofstream f_out(filename);
	f_out << buffer;
	f_out.close();
}

void clean_vertex(tinyobj::attrib_t& attrib, tinyobj::shape_t& shape)
{
	bool has_texcoords = false,has_normal=false;
	if (attrib.texcoords.size() > 0)
		has_texcoords = true;
	if (attrib.normals.size() > 0)
		has_normal = true;
	
	// Find out used vertex, mark true
	std::vector<bool> vertex_used(attrib.vertices.size() / 3, false);
	std::vector<bool> tex_used(attrib.texcoords.size() / 2, false);
	for (size_t face_id = 0; face_id < shape.mesh.num_face_vertices.size(); face_id++)
	{
		size_t index_offset = 3 * face_id;
		tinyobj::index_t idx0 = shape.mesh.indices[index_offset + 0];
		tinyobj::index_t idx1 = shape.mesh.indices[index_offset + 1];
		tinyobj::index_t idx2 = shape.mesh.indices[index_offset + 2];
		vertex_used[idx0.vertex_index] = true;
		vertex_used[idx1.vertex_index] = true;
		vertex_used[idx2.vertex_index] = true;
		if (has_texcoords)
		{
			tex_used[idx0.texcoord_index] = true;
			tex_used[idx1.texcoord_index] = true;
			tex_used[idx2.texcoord_index] = true;
		}
	}
	// Filter out vertex, normals, texcoords
	attrib.vertices.erase(std::remove_if(attrib.vertices.begin(), attrib.vertices.end(), [&](const tinyobj::real_t& idx)
	{
		return vertex_used[(&idx - &*attrib.vertices.begin()) / 3] == false;
	}), attrib.vertices.end());
	if(has_normal)
		attrib.normals.erase(std::remove_if(attrib.normals.begin(), attrib.normals.end(), [&](const tinyobj::real_t& idx)
		{
			return vertex_used[(&idx - &*attrib.normals.begin()) / 3] == false;
		}), attrib.normals.end());
	if (has_texcoords)
		attrib.texcoords.erase(std::remove_if(attrib.texcoords.begin(), attrib.texcoords.end(),
	                                      [&](const tinyobj::real_t& idx)
	                                      {
		                                      return tex_used[(&idx - &*attrib.texcoords.begin()) / 2] == false;
	                                      }), attrib.texcoords.end());

	// Create redirect index map
	std::vector<size_t> vertex_redirect;
	std::vector<size_t> tex_redirect;
	size_t current_vertex_id = 0;
	size_t current_tex_id = 0;
	for (size_t vertex_id = 0; vertex_id < vertex_used.size(); vertex_id++)
	{
		if (vertex_used[vertex_id])
		{
			vertex_redirect.push_back(current_vertex_id + 1);
			current_vertex_id += 1;
		}
		else
			vertex_redirect.push_back(0);
	}
	if(has_texcoords)
		for (size_t tex_id = 0; tex_id < tex_used.size(); tex_id++)
		{
			if (tex_used[tex_id])
			{
				tex_redirect.push_back(current_tex_id + 1);
				current_tex_id += 1;
			}
			else
				tex_redirect.push_back(0);
		}

	// Adjust index from face to vertex according to the vertex_redirect array
	// Also delete duplicated faces
	std::set<std::tuple<int, int, int, int, int, int, int, int, int>> face_already_assigned;
	std::vector<bool> face_should_delete;
	for (size_t face_id = 0; face_id < shape.mesh.num_face_vertices.size(); face_id++)
	{
		if (shape.mesh.num_face_vertices[face_id] != 3)
			throw;
		size_t index_offset = 3 * face_id;
		tinyobj::index_t& idx0 = shape.mesh.indices[index_offset + 0];
		tinyobj::index_t& idx1 = shape.mesh.indices[index_offset + 1];
		tinyobj::index_t& idx2 = shape.mesh.indices[index_offset + 2];

		auto key = std::make_tuple(idx0.vertex_index, idx1.vertex_index, idx2.vertex_index,
			idx0.normal_index, idx1.normal_index, idx2.normal_index,
			idx0.texcoord_index, idx1.texcoord_index, idx2.texcoord_index);

		if (face_already_assigned.insert(key).second)
			face_should_delete.push_back(false);
		else
			face_should_delete.push_back(true);
		
		idx0.vertex_index = vertex_redirect[idx0.vertex_index] - 1;
		idx1.vertex_index = vertex_redirect[idx1.vertex_index] - 1;
		idx2.vertex_index = vertex_redirect[idx2.vertex_index] - 1;
		if(has_normal)
		{
			idx0.normal_index = vertex_redirect[idx0.normal_index] - 1;
			idx1.normal_index = vertex_redirect[idx1.normal_index] - 1;
			idx2.normal_index = vertex_redirect[idx2.normal_index] - 1;
			assert(idx0.normal_index != -1);
			assert(idx1.normal_index != -1);
			assert(idx2.normal_index != -1);
		}
		if(has_texcoords)
		{
			idx0.texcoord_index = tex_redirect[idx0.texcoord_index] - 1;
			idx1.texcoord_index = tex_redirect[idx1.texcoord_index] - 1;
			idx2.texcoord_index = tex_redirect[idx2.texcoord_index] - 1;
			assert(idx0.texcoord_index != -1);
			assert(idx1.texcoord_index != -1);
			assert(idx2.texcoord_index != -1);
		}


		assert(idx0.vertex_index != -1);
		assert(idx1.vertex_index != -1);
		assert(idx2.vertex_index != -1);
		
	}
	shape.mesh.indices.erase(
		std::remove_if(shape.mesh.indices.begin(), shape.mesh.indices.end(),
		               [&,index=0](const tinyobj::index_t& idx) mutable
		               {
			               return face_should_delete[index++ / 3] == true;
		               }),
		shape.mesh.indices.end());
	shape.mesh.material_ids.erase(
		std::remove_if(shape.mesh.material_ids.begin(), shape.mesh.material_ids.end(),
			[&, index = 0](const auto& idx) mutable
	{
		return face_should_delete[index++] == true;
	}),
		shape.mesh.material_ids.end());
	shape.mesh.num_face_vertices.erase(
		std::remove_if(shape.mesh.num_face_vertices.begin(), shape.mesh.num_face_vertices.end(),
			[&, index = 0](const auto& idx) mutable
	{
		return face_should_delete[index++] == true;
	}),
		shape.mesh.num_face_vertices.end());
	
}

void clean_materials(tinyobj::shape_t& shape, std::vector<tinyobj::material_t>& materials)
{
	if (materials.size() == 0)
		return;
	// Find out used vertex, mark true
	std::vector<bool> materials_used(materials.size(), false);
	std::vector<int> materials_reid(materials.size(),-1);

	for (size_t material_id = 0; material_id < shape.mesh.material_ids.size(); material_id++)
	{
		materials_used[shape.mesh.material_ids[material_id]] = true;
	}

	int cur_id_num = 0;
	for(int id_material=0;id_material<materials.size();++id_material)
	{
		if(materials_used[id_material]==true)
		{
			materials_reid[id_material] = cur_id_num;
			cur_id_num += 1;
		}
	}

	for (size_t material_id = 0; material_id < shape.mesh.material_ids.size(); material_id++)
	{
		shape.mesh.material_ids[material_id] = materials_reid[shape.mesh.material_ids[material_id]];
	}
	
	// Filter out vertex, normals, texcoords
	materials.erase(std::remove_if(materials.begin(), materials.end(),
		[&](const tinyobj::material_t& idx)
		{
			return materials_used[(&idx - &*materials.begin())] == false;
		}), materials.end());
}

/*
Get split mesh with a big whole mesh
*/

void merge_obj(const std::string& v_file,
				const std::vector<tinyobj::attrib_t>& v_attribs, const std::vector<std::vector<tinyobj::shape_t>>& saved_shapes,
				const std::vector < std::vector<tinyobj::material_t>>& materials,
			    const int start_id)
{
	FILE* fp = fopen(v_file.c_str(), "w");
	if (!fp)
	{
		fprintf(stderr, "Failed to open file [ %s ] for write.\n", v_file.c_str());
		return;
	}

	boost::filesystem::path output_file(v_file);
	std::string basename = output_file.filename().stem().string() + ".mtl";
	std::string material_filename = (output_file.parent_path() / basename).string();


	fprintf(fp, "mtllib %s\n\n", basename.c_str());
	
	size_t vertex_already_assigned = 0;
	size_t tex_already_assigned = 0;
	std::vector<tinyobj::material_t> total_mtls;
	for (int i_mesh = 0; i_mesh < v_attribs.size(); i_mesh += 1)
	{
		const auto& attributes = v_attribs[i_mesh];
		// vertex
		for (size_t k = 0; k < attributes.vertices.size(); k += 3)
		{
			fprintf(fp, "v %f %f %f\n",
			        attributes.vertices[k + 0],
			        attributes.vertices[k + 1],
			        attributes.vertices[k + 2]);
		}

		fprintf(fp, "\n");

		// normal
		for (size_t k = 0; k < attributes.normals.size(); k += 3)
		{
			fprintf(fp, "vn %f %f %f\n",
			        attributes.normals[k + 0],
			        attributes.normals[k + 1],
			        attributes.normals[k + 2]);
		}

		fprintf(fp, "\n");

		// facevarying texcoord
		for (size_t k = 0; k < attributes.texcoords.size(); k += 2)
		{
			fprintf(fp, "vt %f %f\n",
			        attributes.texcoords[k + 0],
			        attributes.texcoords[k + 1]);
		}
	}
	for (int i_mesh = 0; i_mesh < v_attribs.size(); i_mesh += 1)
	{
		// Mesh
		const auto& attributes = v_attribs[i_mesh];
		const auto& shapes = saved_shapes[i_mesh];

		fprintf(fp, "\n");

		fprintf(fp, "g %s\n", std::to_string(i_mesh + start_id).c_str());

		// face
		int prev_material_id = -1;
		for(int i_shape=0;i_shape< shapes.size();i_shape++)
		{
			const auto& shape = shapes[i_shape];
			int face_index = 0;
			for (size_t k = 0; k < shape.mesh.indices.size(); k += shape.mesh.num_face_vertices[face_index++])
			{
				// Check Materials
				int material_id = shape.mesh.material_ids[face_index];
				if (material_id != prev_material_id)
				{
					std::string material_name = materials[i_mesh][material_id].name;
					fprintf(fp, "usemtl %s\n", material_name.c_str());
					prev_material_id = material_id;
				}

				unsigned char v_per_f = shape.mesh.num_face_vertices[face_index];
				// Imperformant, but if you want to have variable vertices per face, you need some kind of a dynamic loop.
				fprintf(fp, "f");
				for (int l = 0; l < v_per_f; l++)
				{
					const tinyobj::index_t& ref = shape.mesh.indices[k + l];
					// v0/t0/vn0
					fprintf(fp, " %d/%d/%d", ref.vertex_index + 1 + vertex_already_assigned,
						ref.texcoord_index + 1 + tex_already_assigned, ref.normal_index + 1 + vertex_already_assigned);
				}
				fprintf(fp, "\n");
			}
		}
		
		vertex_already_assigned += attributes.vertices.size() / 3;
		tex_already_assigned += attributes.texcoords.size() / 2;
		std::copy(materials[i_mesh].begin(), materials[i_mesh].end(), std::back_inserter(total_mtls));
	}
	fclose(fp);

	// Copy texture
	boost::filesystem::path tex_root = output_file.parent_path() / "mtl";
	if (!boost::filesystem::exists(tex_root))
		boost::filesystem::create_directories(tex_root);
	for (auto& item_mtl : total_mtls)
	{
		if(item_mtl.diffuse_texname!="")
		{
			boost::filesystem::path new_path = tex_root / boost::filesystem::path(item_mtl.diffuse_texname).filename();
			boost::filesystem::copy_file(item_mtl.diffuse_texname,
				new_path,boost::filesystem::copy_option::overwrite_if_exists);
			item_mtl.diffuse_texname = new_path.string();
		}
		if (item_mtl.specular_texname != "")
		{
			boost::filesystem::path new_path = tex_root / boost::filesystem::path(item_mtl.specular_texname).filename();
			boost::filesystem::copy_file(item_mtl.specular_texname,
				new_path, boost::filesystem::copy_option::overwrite_if_exists);
			item_mtl.specular_texname = new_path.string();
		}
		if (item_mtl.normal_texname != "")
		{
			boost::filesystem::path new_path = tex_root / boost::filesystem::path(item_mtl.normal_texname).filename();
			boost::filesystem::copy_file(item_mtl.normal_texname,
				new_path, boost::filesystem::copy_option::overwrite_if_exists);
			item_mtl.normal_texname = new_path.string();
		}
	}
	bool ret = WriteMat(material_filename, total_mtls);

}

void split_obj(const std::string& file_dir, const std::string& file_name, const float resolution,const float v_filter_height, const int obj_max_builidng_num,
	const std::string& output_dir,const int split_axis)
{
	int ref_axis1, ref_axis2;
	if(split_axis==0)
	{
		ref_axis1 = 1;ref_axis2 = 2;
	}
	else if(split_axis == 1)
	{
		ref_axis1 = 0;ref_axis2 = 2;
	}
	else if (split_axis == 2)
	{
		ref_axis1 = 0;ref_axis2 = 1;
	}
	std::cout << "----------Start split obj----------" << std::endl;

	const float Z_THRESHOLD = v_filter_height;
	std::cout << "1/6 Read mesh" << std::endl;
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::tie(attrib, shapes, materials) = load_obj(file_dir + "/" + file_name, true, file_dir);

	// Calculate bounding box
	std::cout << "2/6 Calculate bounding box" << std::endl;
	float min_box[3] = { 1e8 ,1e8 ,1e8 };
	float max_box[3] = { -1e8 ,-1e8 ,-1e8 };
	for (size_t s = 0; s < shapes.size(); s++)
	{
		size_t index_offset = 0;
		for (size_t face_id = 0; face_id < shapes[s].mesh.num_face_vertices.size(); face_id++)
		{
			if (shapes[s].mesh.num_face_vertices[face_id] != 3)
				throw;
			for (size_t v = 0; v < 3; v++)
			{
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
				tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
				tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

				min_box[0] = min_box[0] < vx ? min_box[0] : vx;
				min_box[1] = min_box[1] < vy ? min_box[1] : vy;
				min_box[2] = min_box[2] < vz ? min_box[2] : vz;

				max_box[0] = max_box[0] > vx ? max_box[0] : vx;
				max_box[1] = max_box[1] > vy ? max_box[1] : vy;
				max_box[2] = max_box[2] > vz ? max_box[2] : vz;
			}
			index_offset += 3;
		}
	}

	// Construct height map
	std::cout << "3/6 Construct height map" << std::endl;
	cv::Mat img((int)((max_box[ref_axis2] - min_box[ref_axis2]) / resolution) + 1, (int)((max_box[ref_axis1] - min_box[ref_axis1]) / resolution) + 1, CV_32FC1,
	            cv::Scalar(0.f));
	std::vector<std::vector<std::vector<std::pair<size_t, size_t>>>> records(img.rows,
	                                                                         std::vector<std::vector<std::pair<
		                                                                         size_t, size_t>>>(
		                                                                         img.cols,
		                                                                         std::vector<std::pair<size_t, size_t>
		                                                                         >()));

	for (size_t s = 0; s < shapes.size(); s++)
	{
		for (size_t face_id = 0; face_id < shapes[s].mesh.num_face_vertices.size(); face_id++)
		{
			size_t index_offset = 3 * face_id;

			tinyobj::index_t idx0 = shapes[s].mesh.indices[index_offset + 0];
			tinyobj::index_t idx1 = shapes[s].mesh.indices[index_offset + 1];
			tinyobj::index_t idx2 = shapes[s].mesh.indices[index_offset + 2];

			float vertex0[3] = { attrib.vertices[3 * idx0.vertex_index + 0] ,attrib.vertices[3 * idx0.vertex_index + 1] ,attrib.vertices[3 * idx0.vertex_index + 2] };
			float vertex1[3] = { attrib.vertices[3 * idx1.vertex_index + 0] ,attrib.vertices[3 * idx1.vertex_index + 1] ,attrib.vertices[3 * idx1.vertex_index + 2] };
			float vertex2[3] = { attrib.vertices[3 * idx2.vertex_index + 0] ,attrib.vertices[3 * idx2.vertex_index + 1] ,attrib.vertices[3 * idx2.vertex_index + 2] };

			int xmin_item = (std::min({ vertex0[ref_axis1], vertex1[ref_axis1], vertex2[ref_axis1] }) - min_box[ref_axis1]) / resolution;
			int ymin_item = (std::min({ vertex0[ref_axis2], vertex1[ref_axis2], vertex2[ref_axis2] }) - min_box[ref_axis2]) / resolution;
			int xmax_item = (std::max({ vertex0[ref_axis1], vertex1[ref_axis1], vertex2[ref_axis1] }) - min_box[ref_axis1]) / resolution;
			int ymax_item = (std::max({ vertex0[ref_axis2], vertex1[ref_axis2], vertex2[ref_axis2] }) - min_box[ref_axis2]) / resolution;

			typedef CGAL::Simple_cartesian<int> K;
			CGAL::Triangle_2<K> t1(
				CGAL::Point_2<K>((vertex0[ref_axis1] - min_box[ref_axis1]) / resolution, (vertex0[ref_axis2] - min_box[ref_axis2]) / resolution),
				CGAL::Point_2<K>((vertex1[ref_axis1] - min_box[ref_axis1]) / resolution, (vertex1[ref_axis2] - min_box[ref_axis2]) / resolution),
				CGAL::Point_2<K>((vertex2[ref_axis1] - min_box[ref_axis1]) / resolution, (vertex2[ref_axis2] - min_box[ref_axis2]) / resolution)
			);
			for (int x = xmin_item; x < xmax_item + 1; ++x)
				for (int y = ymin_item; y < ymax_item + 1; ++y)
				{
					if (t1.has_on_bounded_side(CGAL::Point_2<K>(x, y))
						|| t1.has_on_boundary(CGAL::Point_2<K>(x, y)))
					{
						img.at<float>(y, x) = 255;
						records[y][x].push_back(std::make_pair(s, face_id));
					}
				}
		}
	}
	cv::imwrite(output_dir + "/bird_view.jpg", img);

	// Travel to find connect component
	std::cout << "4/6 Travel to find connect component" << std::endl;
	cv::Mat img_traveled(img.rows, img.cols,CV_32FC1, cv::Scalar(0.f));
	struct Building_Cluster
	{
		std::vector<int> xs;
		std::vector<int> ys;
	};
	std::vector<Building_Cluster> buildings;

	std::queue<std::pair<int, int>> node;
	for (int x = 0; x < img.cols; ++x)
	{
		for (int y = 0; y < img.rows; ++y)
		{
			if (img_traveled.at<float>(y, x) == 0)
			{
				Building_Cluster building;
				node.push(std::make_pair(x, y));
				while (!node.empty())
				{
					auto cur = node.front();
					node.pop();
					int cur_x = cur.first;
					int cur_y = cur.second;
					if (cur_x < 0 || cur_y < 0 || cur_x >= img.cols || cur_y >= img.rows)
						continue;;
					if (img.at<float>(cur_y, cur_x) == 0 || img_traveled.at<float>(cur_y, cur_x) != 0)
						continue;;

					building.xs.push_back(cur_x);
					building.ys.push_back(cur_y);
					img_traveled.at<float>(cur_y, cur_x) = 255;
					node.push(std::make_pair(cur_x - 1, cur_y - 1));
					node.push(std::make_pair(cur_x - 1, cur_y));
					node.push(std::make_pair(cur_x - 1, cur_y + 1));
					node.push(std::make_pair(cur_x, cur_y - 1));
					node.push(std::make_pair(cur_x, cur_y + 1));
					node.push(std::make_pair(cur_x + 1, cur_y - 1));
					node.push(std::make_pair(cur_x + 1, cur_y));
					node.push(std::make_pair(cur_x + 1, cur_y + 1));
				}
				if (building.xs.size() > 0)
					buildings.push_back(building);
			}
		}
	}

	std::cout << buildings.size() << " in total\n";

	// Save
	std::cout << "5/6 Save splited model" << std::endl;
	std::vector<tinyobj::shape_t> saved_shapes;
	std::vector<tinyobj::attrib_t> saved_attrib;
	int building_num = 0;
	int obj_num = 0;
	for (int building_idx = 0; building_idx < buildings.size(); building_idx++)
	{
		tinyobj::attrib_t cur_attr = tinyobj::attrib_t(attrib);
		std::vector<tinyobj::material_t> item_mtl(materials);
		tinyobj::shape_t cur_shape;

		int area_2d = buildings[building_idx].xs.size();
		if (area_2d <= 1)
			continue;

		int x_center = *std::max_element(buildings[building_idx].xs.begin(), buildings[building_idx].xs.end()) + (*
			std::min_element(buildings[building_idx].xs.begin(), buildings[building_idx].xs.end()));
		int y_center = *std::max_element(buildings[building_idx].ys.begin(), buildings[building_idx].ys.end()) + (*
			std::min_element(buildings[building_idx].ys.begin(), buildings[building_idx].ys.end()));
		x_center = x_center / 2 * resolution + min_box[ref_axis1];
		y_center = y_center / 2 * resolution + min_box[ref_axis2];

		std::map<Eigen::VectorXf, int> vertex_already_assigned;

		for (int pixel_id = 0; pixel_id < area_2d; pixel_id += 1)
		{
			int x = buildings[building_idx].xs[pixel_id];
			int y = buildings[building_idx].ys[pixel_id];

			for (const auto mesh_id : records[y][x])
			{
				tinyobj::index_t idx0 = shapes[mesh_id.first].mesh.indices[mesh_id.second * 3 + 0];
				tinyobj::index_t idx1 = shapes[mesh_id.first].mesh.indices[mesh_id.second * 3 + 1];
				tinyobj::index_t idx2 = shapes[mesh_id.first].mesh.indices[mesh_id.second * 3 + 2];

				cur_shape.mesh.num_face_vertices.push_back(3);
				cur_shape.mesh.material_ids.push_back(shapes[mesh_id.first].mesh.material_ids[mesh_id.second]);
				cur_shape.mesh.indices.push_back(idx0);
				cur_shape.mesh.indices.push_back(idx1);
				cur_shape.mesh.indices.push_back(idx2);
			}
		}

		cur_shape.name = std::to_string(building_num);
		clean_vertex(cur_attr, cur_shape);
		clean_materials(cur_shape, item_mtl);
		bool preserve_flag = false;
		for (int i = 0; i < cur_attr.vertices.size() / 3; ++i)
		{
			float cur_z = cur_attr.vertices[i * 3 + split_axis];
			if (cur_z > Z_THRESHOLD)
			{
				preserve_flag = true;
				break;
			}
		}
		if (!preserve_flag)
			continue;

		saved_shapes.push_back(cur_shape);
		saved_attrib.push_back(cur_attr);

		if (obj_max_builidng_num > 0 && saved_shapes.size() >= obj_max_builidng_num)
		{
			std::cout << "5.5/6 Save max num split obj" << std::endl;
			merge_obj(output_dir + "/" + "total_split" + std::to_string(obj_num) + ".obj", saved_attrib, 
				std::vector < std::vector<tinyobj::shape_t>>{ saved_shapes },
				std::vector < std::vector<tinyobj::material_t>>{ materials},
				building_num - obj_max_builidng_num + 1);
			std::cout << "----------Partial Save done----------" << std::endl << std::endl;
			saved_shapes.clear();
			saved_attrib.clear();
			obj_num++;
		}

		for (int i_vertex = 0; i_vertex < cur_attr.vertices.size(); i_vertex += 1)
		{
			if (i_vertex % 3 == ref_axis1)
				cur_attr.vertices[i_vertex] -= x_center;
			else if (i_vertex % 3 == ref_axis2)
				cur_attr.vertices[i_vertex] -= y_center;
		}

		std::ofstream f_out(output_dir + "/" + std::to_string(building_num) + ".txt");
		f_out << x_center << "," << y_center << std::endl;
		f_out.close();

		write_obj(output_dir + "/" + std::to_string(building_num) + ".obj", cur_attr,
		          std::vector<tinyobj::shape_t>{cur_shape}, materials);
		building_num += 1;
	}
	std::cout << "6/6 Save whole split obj" << std::endl;
	merge_obj(output_dir + "/" + "total_split" + std::to_string(obj_num) + ".obj", saved_attrib, 
		std::vector < std::vector<tinyobj::shape_t>>{ saved_shapes },
		std::vector < std::vector<tinyobj::material_t>>{ materials},
		building_num - saved_attrib.size() + 1);
	std::cout << "----------Split obj done----------" << std::endl << std::endl;

}

void rename_material(const std::string& file_dir, const std::string& file_name, const std::string& v_output_dir)
{
	std::cout << "----------Start rename material----------" << std::endl;

	boost::filesystem::path output_root(v_output_dir);
	boost::system::error_code ec;
	try
	{
		if (boost::filesystem::exists(output_root))
			boost::filesystem::remove_all(output_root, ec);
		boost::filesystem::create_directories(v_output_dir);
		boost::filesystem::create_directories(v_output_dir + "/mat");
	}
	catch (...)
	{
		std::cout << ec << std::endl;
	}

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::tie(attrib, shapes, materials) = load_obj(file_dir + "/" + file_name, true, file_dir);
	std::map<std::string, std::string> texture_set;
	for (int i = 0; i < materials.size(); ++i)
	{
		//continue;
		auto& material = materials[i];

		if (material.diffuse_texname.size() <= 2)
		{
			//std::cout << material.name << std::endl;
			continue;
		}
		material.name = (boost::format("m_%d") % i).str();


		std::string img_name_old = material.diffuse_texname;
		std::string img_name_new;
		if (texture_set.find(img_name_old) != texture_set.end())
		{
			img_name_new = texture_set.at(img_name_old);
		}
		else
		{
			std::string extension_name = boost::filesystem::path(img_name_old).extension().string();

			img_name_new = (boost::format("mat/tex_%s%s") % texture_set.size() % extension_name).str();
			texture_set.insert(std::make_pair(img_name_old, img_name_new));

			boost::filesystem::path img_path_old(file_dir);
			img_path_old = img_path_old / img_name_old;
			if (!boost::filesystem::exists(img_path_old))
				throw;
			boost::filesystem::copy_file(img_path_old, output_root / img_name_new,
			                             boost::filesystem::copy_option::overwrite_if_exists);
		}
		material.diffuse_texname = img_name_new;
		if (material.ambient_texname == img_name_old)
			material.ambient_texname = img_name_new;
	}
	write_obj((output_root / "renamed_material.obj").string(), attrib, shapes, materials);
	std::cout << "----------Rename material done----------" << std::endl << std::endl;

	return;
}

std::vector<tinyobj::shape_t> split_obj_according_to_footprint(const tinyobj::attrib_t& v_attribs, const std::vector<tinyobj::shape_t>& v_shapes, const std::vector<Proxy>& v_proxys,const float v_edge_threshold)
{
	std::vector<tinyobj::shape_t> out_shapes(v_proxys.size());
	tinyobj::shape_t ground_shape;

	std::vector<Polygon_2> proxys;
	//Dilate footprint
	for (int i_proxy = 0; i_proxy < v_proxys.size(); i_proxy++) {
		Proxy proxy = v_proxys[i_proxy];
		
		//if (!proxy.polygon.is_counterclockwise_oriented())
		//{
		//	std::cout << "Is not counterclockwise" << std::endl;
		//	throw;
		//}
		Polygon_2 p;
		auto start = proxy.polygon.vertices_circulator();
		auto end = proxy.polygon.vertices_circulator();
		
		do
		{
			CGAL::Vector_2<K> seg_prev = *start-*(start-1);
			CGAL::Vector_2<K> seg_next = *(start + 1) - *start;
			
			CGAL::Vector_2<K> normal;
			if(seg_next.direction().counterclockwise_in_between(seg_prev.direction(),-seg_prev.direction()))
			{
				seg_prev /= (std::sqrt(seg_prev.squared_length())+1e-6);
				seg_next /= (std::sqrt(seg_next.squared_length()) + 1e-6);
				normal = (seg_prev - seg_next);
			}
			else
			{
				seg_prev /= (std::sqrt(seg_prev.squared_length()) + 1e-6);
				seg_next /= (std::sqrt(seg_next.squared_length()) + 1e-6);
				normal = (-seg_prev + seg_next);
			}
			normal /= std::sqrt((normal.squared_length()) + 1e-6);

			Point_2 new_point = *start + normal * v_edge_threshold;
			p.push_back(new_point);
			//p.push_back(*start);
		} while (++start != end);
		proxys.push_back(p);
		std::cout << p << std::endl;
		std::cout << proxy.polygon << std::endl;
	}
	
	for (int i_shape = 0; i_shape < v_shapes.size(); ++i_shape)
	{
		const tinyobj::mesh_t& shape = v_shapes[i_shape].mesh;
		size_t num_faces = shape.num_face_vertices.size();
		for (size_t face_id = 0; face_id < num_faces; face_id++) {
			size_t index_offset = 3 * face_id;

			tinyobj::index_t idx0 = shape.indices[index_offset + 0];
			tinyobj::index_t idx1 = shape.indices[index_offset + 1];
			tinyobj::index_t idx2 = shape.indices[index_offset + 2];
			CGAL::Point_2<K> point1(v_attribs.vertices[3 * idx0.vertex_index + 0], v_attribs.vertices[3 * idx0.vertex_index + 1]);
			CGAL::Point_2<K> point2(v_attribs.vertices[3 * idx1.vertex_index + 0], v_attribs.vertices[3 * idx1.vertex_index + 1]);
			CGAL::Point_2<K> point3(v_attribs.vertices[3 * idx2.vertex_index + 0], v_attribs.vertices[3 * idx2.vertex_index + 1]);

			std::vector<int> preserved;
			//int preserved = -1;
			for (int i_proxy = 0; i_proxy < v_proxys.size(); i_proxy++) {
				//const Polygon_2& polygon = v_proxys[i_proxy].polygon;
				const Polygon_2& polygon = proxys[i_proxy];
				
				if(polygon.bounded_side(point1) == CGAL::Bounded_side::ON_BOUNDED_SIDE||
					polygon.bounded_side(point2) == CGAL::Bounded_side::ON_BOUNDED_SIDE||
					polygon.bounded_side(point3) == CGAL::Bounded_side::ON_BOUNDED_SIDE)
				{
					preserved.push_back(i_proxy);
					//preserved = i_proxy;
					break;
				}
			}

			tinyobj::mesh_t* mesh;

			if (preserved.size()==0)
			{
				mesh = &ground_shape.mesh;
				mesh->material_ids.push_back(shape.material_ids[face_id]);
				mesh->num_face_vertices.push_back(shape.num_face_vertices[face_id]);
				mesh->indices.push_back(shape.indices[3 * face_id + 0]);
				mesh->indices.push_back(shape.indices[3 * face_id + 1]);
				mesh->indices.push_back(shape.indices[3 * face_id + 2]);
			}
			else
				for(auto item:preserved)
				{
					mesh = &out_shapes[item].mesh;
					mesh->material_ids.push_back(shape.material_ids[face_id]);
					mesh->num_face_vertices.push_back(shape.num_face_vertices[face_id]);
					mesh->indices.push_back(shape.indices[3 * face_id + 0]);
					mesh->indices.push_back(shape.indices[3 * face_id + 1]);
					mesh->indices.push_back(shape.indices[3 * face_id + 2]);
				}
		}
	}
	out_shapes.push_back(ground_shape);

	return out_shapes;
}

/*
	xmin xmax ymin ymax zmax in sequence
*/
std::vector<float> get_bounds(std::string path, float v_bounds)
{
	std::vector<float> output;
	std::ifstream ObjFile(path);
	std::string line;
	float xmin = 99999;
	float xmax = -99999;
	float ymin = 99999;
	float ymax = -99999;
	float zmax = -99999;
	while (getline(ObjFile, line))
	{
		std::vector<std::string> vData;
		if (line.substr(0, line.find(" ")) == "v")
		{
			line = line.substr(line.find(" ") + 1);
			for (int i = 0; i < 3; i++)
			{
				vData.push_back(line.substr(0, line.find(" ")));
				line = line.substr(line.find(" ") + 1);
			}
			vData[2] = vData[2].substr(0, vData[2].find("\n"));
			float x = atof(vData[0].c_str());
			float y = atof(vData[1].c_str());
			float z = atof(vData[2].c_str()) + v_bounds;
			xmin = std::min(x, xmin);
			xmax = std::max(x, xmax);
			ymin = std::min(y, ymin);
			ymax = std::max(y, ymax);
			zmax = std::max(z, zmax);
		}
	}
	output.push_back(xmin);
	output.push_back(xmax);
	output.push_back(ymin);
	output.push_back(ymax);
	output.push_back(zmax);
	return output;
}


std::vector<Polygon_2> get_polygons(std::string file_path)
{
	std::ifstream inputFile;
	inputFile.open(file_path);
	if (!inputFile)
	{
		std::cout << "can't find file" << std::endl;
	}
	std::vector<std::vector<float>> temp_data;
	while (!inputFile.eof())
	{
		std::string x_temp, y_temp, index;
		inputFile >> x_temp >> y_temp >> index;
		temp_data.push_back(std::vector<float>{float(atof(x_temp.c_str())), float(atof(y_temp.c_str())), float(atof(index.c_str()))});
	}
	inputFile.close();
	float now_index = temp_data[0][2];
	std::vector<Polygon_2> polygon_vector;
	Polygon_2 polygon;
	for (auto point : temp_data)
	{
		if (point[2] == now_index)
		{
			polygon.push_back(Point_2(point[0], point[1]));
		}
		else
		{
			if (polygon.is_simple())
				polygon_vector.push_back(polygon);
			polygon = Polygon_2();
			polygon.push_back(Point_2(point[0], point[1]));
			now_index = point[2];
		}
	}
	return polygon_vector;
}

Point_set sample_points(const Surface_mesh& v_mesh, const int v_num_points)
{
	std::mt19937 gen; std::uniform_real_distribution<double> dist(0.0f, 1.0f);
	Point_set o_point_set(true);
	double total_area = CGAL::Polygon_mesh_processing::area(v_mesh);
	double point_per_area = (double)v_num_points / total_area;

#pragma omp parallel for
	for(int i_face = 0;i_face<v_mesh.num_faces();++i_face)
	{
		const auto it_face =*(v_mesh.faces_begin() + i_face);
		Point_3 vertexes[3];
		int i_vertex=0;
		for(auto it_vertex = (v_mesh.vertices_around_face(v_mesh.halfedge(it_face))).begin();it_vertex!=(v_mesh.vertices_around_face(v_mesh.halfedge(it_face))).end();++it_vertex)
		{
			vertexes[i_vertex++]=v_mesh.point(*it_vertex);
		}
		
        Vector_3 normal = CGAL::cross_product(vertexes[1] - vertexes[0],vertexes[2] - vertexes[0]);
		normal/=std::sqrt(normal.squared_length());

		double area = CGAL::Polygon_mesh_processing::face_area(it_face,v_mesh);

        double face_samples = area * point_per_area;
        uint num_face_samples = face_samples;

        if (dist(gen) < (face_samples - static_cast<double>(num_face_samples))) {
            num_face_samples += 1;
        }

        for (uint j = 0; j < num_face_samples; ++j) {
            double r1 = dist(gen);
            double r2 = dist(gen);

            double tmp = std::sqrt(r1);
            double u = 1.0f - tmp;
            double v = r2 * tmp;

            double w = 1.0f - v - u;
			auto point = Point_3(
				u*vertexes[0].x()+v*vertexes[1].x()+w*vertexes[2].x(),
				u*vertexes[0].y()+v*vertexes[1].y()+w*vertexes[2].y(),
				u*vertexes[0].z()+v*vertexes[1].z()+w*vertexes[2].z()
			) ;
#pragma omp critical
        	{
			o_point_set.insert(point,normal);
			}
        }
	}
	return o_point_set;
}

Point_set sample_points_according_density(const std::vector<Triangle_3>& v_mesh, const float v_num_points_per_m2)
{
	double total_area = 0.;
#pragma omp parallel for reduction(+:total_area)
	for (int i_face = 0;i_face < v_mesh.size();++i_face)
		total_area += std::sqrt(v_mesh[i_face].squared_area());
	return sample_points(v_mesh, v_num_points_per_m2 * total_area);
}

Point_set sample_points(const std::vector<Triangle_3>& v_mesh, const int v_num_points)
{
	std::mt19937 gen; std::uniform_real_distribution<double> dist(0.0f, 1.0f);
	Point_set o_point_set(true);

	double total_area = 0.;
#pragma omp parallel for reduction(+:total_area)
	for(int i_face = 0;i_face<v_mesh.size();++i_face)
		total_area+=std::sqrt(v_mesh[i_face].squared_area());

	double point_per_area = (double)v_num_points / total_area;

#pragma omp parallel for
	for(int i_face = 0;i_face<v_mesh.size();++i_face)
	{
		Point_3 vertexes[3];
		vertexes[0]=v_mesh[i_face].vertex(0);
		vertexes[1]=v_mesh[i_face].vertex(1);
		vertexes[2]=v_mesh[i_face].vertex(2);
		
		
        Vector_3 normal = CGAL::cross_product(vertexes[1] - vertexes[0],vertexes[2] - vertexes[0]);
		normal/=std::sqrt(normal.squared_length());

		double area = std::sqrt(v_mesh[i_face].squared_area());

        double face_samples = area * point_per_area;
        uint num_face_samples = face_samples;

        if (dist(gen) < (face_samples - static_cast<double>(num_face_samples))) {
            num_face_samples += 1;
        }

        for (uint j = 0; j < num_face_samples; ++j) {
            double r1 = dist(gen);
            double r2 = dist(gen);

            double tmp = std::sqrt(r1);
            double u = 1.0f - tmp;
            double v = r2 * tmp;

            double w = 1.0f - v - u;
			auto point = Point_3(
				u*vertexes[0].x()+v*vertexes[1].x()+w*vertexes[2].x(),
				u*vertexes[0].y()+v*vertexes[1].y()+w*vertexes[2].y(),
				u*vertexes[0].z()+v*vertexes[1].z()+w*vertexes[2].z()
			) ;
#pragma omp critical
        	{
			o_point_set.insert(point,normal);
			}
        }
	}
	return o_point_set;
}

Point_set sample_points(const Polyhedron_3& v_mesh, const int v_num_points)
{
	Point_set points;
	CGAL::Random_points_in_triangle_mesh_3<Polyhedron_3> g(v_mesh);
	std::copy_n(g, v_num_points, points.point_back_inserter());
	return points;
}

Point_set sample_points_according_density(const Polyhedron_3& v_mesh, const float v_num_points_per_m2)
{
	double total_area = CGAL::Polygon_mesh_processing::area(v_mesh);
	return sample_points(v_mesh, v_num_points_per_m2 * total_area);
}


float point_box_distance_eigen(const Eigen::Vector2f& v_pos, const Eigen::AlignedBox2f& v_box) {
	float sqDist = 0.0f;
	float x = v_pos.x();
	if (x < v_box.min().x()) sqDist += (v_box.min().x() - x) * (v_box.min().x() - x);
	if (x > v_box.max().x()) sqDist += (x - v_box.max().x()) * (x - v_box.max().x());

	float y = v_pos.y();
	if (y < v_box.min().y()) sqDist += (v_box.min().y() - y) * (v_box.min().y() - y);
	if (y > v_box.max().y()) sqDist += (y - v_box.max().y()) * (y - v_box.max().y());
	return std::sqrt(sqDist);
}


bool inside_box(const Eigen::Vector2f& v_pos, const Eigen::AlignedBox2f& v_box) {
	return v_pos.x() >= v_box.min().x() && v_pos.x() <= v_box.max().x() && v_pos.y() >= v_box.min().y() && v_pos.y() <= v_box.max().y();
}

Surface_mesh get_box_mesh(const std::vector<Eigen::AlignedBox3f>& v_boxes)
{
	Surface_mesh mesh;

	for(const auto& item: v_boxes)
	{
		auto v0=mesh.add_vertex(Point_3(item.min().x(), item.min().y(), item.min().z()));
		auto v1=mesh.add_vertex(Point_3(item.min().x(), item.max().y(), item.min().z()));
		auto v2=mesh.add_vertex(Point_3(item.max().x(), item.max().y(), item.min().z()));
		auto v3=mesh.add_vertex(Point_3(item.max().x(), item.min().y(), item.min().z()));
		auto v4 = mesh.add_vertex(Point_3(item.min().x(), item.min().y(), item.max().z()));
		auto v5 = mesh.add_vertex(Point_3(item.min().x(), item.max().y(), item.max().z()));
		auto v6 = mesh.add_vertex(Point_3(item.max().x(), item.max().y(), item.max().z()));
		auto v7 = mesh.add_vertex(Point_3(item.max().x(), item.min().y(), item.max().z()));

		mesh.add_face(v0, v1, v2,v3);
		mesh.add_face(v4, v7, v6,v5);
		mesh.add_face(v0, v4, v5,v1);
		mesh.add_face(v3, v2, v6,v7);
		mesh.add_face(v0, v3, v7,v4);
		mesh.add_face(v1, v5, v6,v2);
	}
	return mesh;
}

Surface_mesh get_rotated_box_mesh(const std::vector<Rotated_box>& v_boxes)
{
	Surface_mesh mesh;

	for (const auto& item : v_boxes)
	{
		cv::Point2f cv_points[4];
		item.cv_box.points(cv_points);
		auto v0 = mesh.add_vertex(Point_3(cv_points[0].x, cv_points[0].y, item.box.min().z()));
		auto v1 = mesh.add_vertex(Point_3(cv_points[1].x, cv_points[1].y, item.box.min().z()));
		auto v2 = mesh.add_vertex(Point_3(cv_points[2].x, cv_points[2].y, item.box.min().z()));
		auto v3 = mesh.add_vertex(Point_3(cv_points[3].x, cv_points[3].y, item.box.min().z()));
		
		auto v4 = mesh.add_vertex(Point_3(cv_points[0].x, cv_points[0].y, item.box.max().z()));
		auto v5 = mesh.add_vertex(Point_3(cv_points[1].x, cv_points[1].y, item.box.max().z()));
		auto v6 = mesh.add_vertex(Point_3(cv_points[2].x, cv_points[2].y, item.box.max().z()));
		auto v7 = mesh.add_vertex(Point_3(cv_points[3].x, cv_points[3].y, item.box.max().z()));

		mesh.add_face(v3, v2, v1, v0);
		mesh.add_face(v5, v6, v7, v4);
		mesh.add_face(v1, v5, v4, v0);
		mesh.add_face(v7, v6, v2, v3);
		mesh.add_face(v4, v7, v3, v0);
		mesh.add_face(v2, v6, v5, v1);
	}
	return mesh;
}


void get_box_mesh_with_colors(const std::vector<Eigen::AlignedBox3f>& v_boxes,
	const std::vector<cv::Vec3b>& v_colors,const std::string& v_name) {
	std::ofstream f(v_name);
	int index = 0;
	for (const auto& item : v_boxes) {
		f << (boost::format("v %s %s %s %s %s %s\n") % item.min().x() % item.min().y() % item.min().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.min().x() % item.max().y() % item.min().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.max().x() % item.max().y() % item.min().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.max().x() % item.min().y() % item.min().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.min().x() % item.min().y() % item.max().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.min().x() % item.max().y() % item.max().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.max().x() % item.max().y() % item.max().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();
		f << (boost::format("v %s %s %s %s %s %s\n") % item.max().x() % item.min().y() % item.max().z() % ((float)v_colors[index][2] / 255.f) % ((float)v_colors[index][1] / 255.f) % ((float)v_colors[index][0] / 255.f)).str();

		f << (boost::format("f %s %s %s %s\n")% (index * 8 + 1 +0) % (index * 8 + 1 + 1) % (index * 8 + 1 + 2) % (index * 8 + 1 + 3)).str();
		f << (boost::format("f %s %s %s %s\n")% (index * 8 + 1 +4) % (index * 8 + 1 + 7) % (index * 8 + 1 + 6) % (index * 8 + 1 + 5)).str();
		f << (boost::format("f %s %s %s %s\n")% (index * 8 + 1 +0) % (index * 8 + 1 + 4) % (index * 8 + 1 + 5) % (index * 8 + 1 + 1)).str();
		f << (boost::format("f %s %s %s %s\n")% (index * 8 + 1 +3) % (index * 8 + 1 + 2) % (index * 8 + 1 + 6) % (index * 8 + 1 + 7)).str();
		f << (boost::format("f %s %s %s %s\n")% (index * 8 + 1 +0) % (index * 8 + 1 + 3) % (index * 8 + 1 + 7) % (index * 8 + 1 + 4)).str();
		f << (boost::format("f %s %s %s %s\n")% (index * 8 + 1 +1) % (index * 8 + 1 + 5) % (index * 8 + 1 + 6) % (index * 8 + 1 + 2)).str();
		index += 1;
	}
	f.close();
	return;
}