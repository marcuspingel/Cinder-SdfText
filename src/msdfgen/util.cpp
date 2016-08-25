#include "msdfgen/util.h"

#include "ft2build.h"
#include FT_FREETYPE_H

#define REQUIRE(cond) { if (!(cond)) return false; }

namespace msdfgen {

bool getFontScale(double &output, FT_Face face) {
    output = face->units_per_EM/64.;
    return true;
}

bool getFontWhitespaceWidth(double &spaceAdvance, double &tabAdvance, FT_Face face) {
    FT_Error error = FT_Load_Char(face, ' ', FT_LOAD_NO_SCALE);
    if (error)
        return false;
    spaceAdvance = face->glyph->advance.x/64.;
    error = FT_Load_Char(face, '\t', FT_LOAD_NO_SCALE);
    if (error)
        return false;
    tabAdvance = face->glyph->advance.x/64.;
    return true;
}
	

bool loadGlyph(Shape &output, FT_Face face, unsigned int glyphIndex, double *advance) {
    enum PointType {
        NONE = 0,
        PATH_POINT,
        QUADRATIC_POINT,
        CUBIC_POINT,
        CUBIC_POINT2
    };
	
	auto pointTypeOutput = []( PointType type ) {
		switch ( type ) {
			case NONE: return "NONE"; break;
			case PATH_POINT: return "PATH_POINT"; break;
			case QUADRATIC_POINT: return "QUADRATIC_POINT"; break;
			case CUBIC_POINT: return "CUBIC_POINT"; break;
			case CUBIC_POINT2: return "CUBIC_POINT2"; break;
			default: return "unknown"; break;
		}
	};
	
	if (nullptr == face)
		return false;
	FT_Error error = FT_Load_Glyph(face, glyphIndex, FT_LOAD_NO_SCALE);
	if (error)
		return false;
	output.contours.clear();
	output.inverseYAxis = false;
	if (advance)
		*advance = face->glyph->advance.x/64.;
	
	float glyphScale = 2048.0f / face->units_per_EM;
	
	int last = -1;
//	if ( glyphIndex == 83 || glyphIndex == 51 ) {
//		std::cout << "-------------------------------------------" << std::endl;
//		std::cout << "Num contours: " << face->glyph->outline.n_contours << " glyph: " << glyphIndex <<  std::endl;
//	}
	// For each contour
	for (int i = 0; i < face->glyph->outline.n_contours; ++i) {
		
		Contour &contour = output.addContour();
		int first = last+1;
		int firstPathPoint = -1;
		last = face->glyph->outline.contours[i];
		
		PointType state = NONE;
		Point2 startPoint;
		Point2 controlPoint[2];
//		if ( glyphIndex == 83 || glyphIndex == 51 )
//			std::cout << "Outer For Loop: " << i << std::endl;
		// For each point on the contour
		for (int round = 0, index = first; round == 0; ++index) {
//			if ( glyphIndex == 83 || glyphIndex == 51 )
//				std::cout << "round: " << round << " index: " << index << " first: " << first << " last: " << last << std::endl;
			if (index > last) {
				if (!(firstPathPoint >= 0))
					return false;
				index = first;
			}
			// Close contour
			if (index == firstPathPoint)
				++round;
			
			Point2 point( glyphScale * face->glyph->outline.points[index].x/64., glyphScale * face->glyph->outline.points[index].y/64.);
			PointType pointType = face->glyph->outline.tags[index]&1 ? PATH_POINT : face->glyph->outline.tags[index]&2 ? CUBIC_POINT : QUADRATIC_POINT;
			
			switch (state) {
				case NONE:
					if (pointType == PATH_POINT) {
						firstPathPoint = index;
						startPoint = point;
						state = PATH_POINT;
					}
					
//					if ( glyphIndex == 83 || glyphIndex == 51 )
//						std::cout << "state: " << pointTypeOutput( state ) << " pointType: " << pointTypeOutput( pointType ) << std::endl;
					break;
				case PATH_POINT:
					if (pointType == PATH_POINT) {
						contour.addEdge(new LinearSegment(startPoint, point));
						startPoint = point;
					} else {
						controlPoint[0] = point;
						state = pointType;
					}
//					if ( glyphIndex == 83 || glyphIndex == 51 )
//						std::cout << "state: " << pointTypeOutput( state ) << " pointType: " << pointTypeOutput( pointType ) << std::endl;
					break;
				case QUADRATIC_POINT:
					if (!(pointType != CUBIC_POINT))
						return false;
					if (pointType == PATH_POINT) {
						contour.addEdge(new QuadraticSegment(startPoint, controlPoint[0], point));
						startPoint = point;
						state = PATH_POINT;
					} else {
						Point2 midPoint = .5*controlPoint[0]+.5*point;
						contour.addEdge(new QuadraticSegment(startPoint, controlPoint[0], midPoint));
						startPoint = midPoint;
						controlPoint[0] = point;
					}
					if ( glyphIndex == 83 || glyphIndex == 51 )
						std::cout << "state: " << pointTypeOutput( state ) << " pointType: " << pointTypeOutput( pointType ) << std::endl;
					break;
				case CUBIC_POINT:
					if (!(pointType == CUBIC_POINT))
						return false;;
					controlPoint[1] = point;
					state = CUBIC_POINT2;
//					if ( glyphIndex == 83 || glyphIndex == 51 )
//						std::cout << "state: " << pointTypeOutput( state ) << " pointType: " << pointTypeOutput( pointType ) << std::endl;
					break;
				case CUBIC_POINT2:
					if (!(pointType != QUADRATIC_POINT))
						return false;
					if (pointType == PATH_POINT) {
						contour.addEdge(new CubicSegment(startPoint, controlPoint[0], controlPoint[1], point));
						startPoint = point;
					} else {
						Point2 midPoint = .5*controlPoint[1]+.5*point;
						contour.addEdge(new CubicSegment(startPoint, controlPoint[0], controlPoint[1], midPoint));
						startPoint = midPoint;
						controlPoint[0] = point;
					}
					state = pointType;
//					if ( glyphIndex == 83 || glyphIndex == 51 )
//						std::cout << "state: " << pointTypeOutput( state ) << " pointType: " << pointTypeOutput( pointType ) << std::endl;
					break;
			}
			
		}
	}
	return true;
}

bool loadChar(Shape &output, FT_Face face, unsigned int charCode, double *advance) {
	unsigned int glyphIndex = FT_Get_Char_Index(face, charCode);
	return loadGlyph(output, face, glyphIndex, advance );
}

bool getKerning(double &output, FT_Face face, int charCode1, int charCode2) {
    FT_Vector kerning;
    if (FT_Get_Kerning(face, FT_Get_Char_Index(face, charCode1), FT_Get_Char_Index(face, charCode2), FT_KERNING_UNSCALED, &kerning)) {
        output = 0;
        return false;
    }
    output = kerning.x/64.;
    return true;
}

} // namespace msdfgen